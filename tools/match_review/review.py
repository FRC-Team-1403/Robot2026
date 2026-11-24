"""Generate local match reports from AdvantageKit WPILOG files (Python 3.9+)."""
import argparse
import hashlib
import html
import json
import math
import os
from pathlib import Path
import time
import urllib.request

from wpilog import records
import view

ENABLED = '/DriverStation/Enabled'
AUTO = '/DriverStation/Autonomous'
TEST = '/DriverStation/Test'
BROWN = '/SystemStats/BrownedOut'
VOLTAGE = '/PowerDistribution/Voltage'
CYCLE = '/RealOutputs/LoggedRobot/FullCycleMS'
TARGET = '/RealOutputs/Flywheel/Target RPM'
ACTUAL = '/RealOutputs/Flywheel/Leader RPM'
GYRO = '/RealOutputs/Health/GyroConnected'
UNITS = {VOLTAGE: 'V', CYCLE: 'ms', TARGET: 'RPM', ACTUAL: 'RPM',
         '/RealOutputs/Health/BatteryVolts': 'V', '/DriverStation/MatchTime': 's'}


def finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def load(path):
    rows = list(records(path))
    if not rows:
        raise ValueError('No supported primitive records in WPILOG')
    rows.sort(key=lambda r: r[0])
    return rows


def sessions(rows):
    """Group enables separated by <= 30 s; longer gaps are separate sessions.

    Boundaries are inferred, because older logs have no session identifiers.
    """
    enables = [(t, bool(v)) for t, n, _, v in rows if n == ENABLED]
    spans = []
    start = None
    for t, enabled in enables:
        if enabled and start is None:
            start = t
        elif not enabled and start is not None:
            spans.append((start, t, False))
            start = None
    if start is not None:
        spans.append((start, rows[-1][0], True))
    if not spans:
        return [(rows[0][0], rows[-1][0], True)]
    merged = []
    for start, end, partial in spans:
        if merged and start - merged[-1][1] <= 30:
            merged[-1] = (merged[-1][0], end, partial)
        else:
            merged.append((start, end, partial))
    return merged


def analyze(rows, start, end, partial, session_id, filename):
    # Carry forward last values before the session; AdvantageKit logs changes only.
    state = {}
    timeline = []
    points = {name: [] for name in (VOLTAGE, CYCLE, TARGET, ACTUAL,
              '/RealOutputs/Health/BatteryVolts')}
    incidents = []
    prev_mode = None
    last_fault = False
    gyro_connected = None
    for t, name, _, value in rows:
        if t >= start:
            break
        state[name] = value
    for name in points:
        if finite(state.get(name)):
            points[name].append([start, state[name]])
    # Show the carried-forward state at the first enabled timestamp.
    prev_mode = ('Disabled' if not state.get(ENABLED, False) else
                 'Test' if state.get(TEST, False) else
                 'Autonomous' if state.get(AUTO, False) else 'Teleop')
    for t, name, kind, value in rows:
        if t > end:
            break
        state[name] = value
        if t < start:
            continue
        if name in points and finite(value):
            points[name].append([t, value])
        if name in (ENABLED, AUTO, TEST):
            mode = ('Disabled' if not state.get(ENABLED, False) else
                    'Test' if state.get(TEST, False) else
                    'Autonomous' if state.get(AUTO, False) else 'Teleop')
            if mode != prev_mode:
                timeline.append({'raw_time_s': t, 'elapsed_s': t-start, 'mode': mode})
                prev_mode = mode
        if name in (BROWN, '/RealOutputs/Health/BrownedOut'):
            fault = bool(value)
            if fault and not last_fault:
                incidents.append({'id': f'I{len(incidents)+1}', 'subsystem': 'Health',
                    'observation': 'Brownout reported by robot controller.',
                    'raw_time_s': t, 'window_s': [max(start, t-2), min(end, t+3)],
                    'signals': [name], 'severity': 'high'})
            last_fault = fault
        if name == GYRO:
            connected = bool(value)
            if not connected and gyro_connected is not False:
                incidents.append({'id': f'I{len(incidents)+1}', 'subsystem': 'Drivetrain',
                    'observation': 'The gyro reported that it was disconnected.',
                    'raw_time_s': t, 'window_s': [max(start, t-2), min(end, t+3)],
                    'signals': [name], 'severity': 'high'})
            gyro_connected = connected
    volts = points['/RealOutputs/Health/BatteryVolts'] or points[VOLTAGE]
    # Ignore zero at startup: it commonly means the power distribution has not reported yet.
    valid_volts = [(t, v) for t, v in volts if v > 0]
    if valid_volts:
        t, v = min(valid_volts, key=lambda p: p[1])
        if v < 7:
            signal = '/RealOutputs/Health/BatteryVolts' if points['/RealOutputs/Health/BatteryVolts'] else VOLTAGE
            incidents.append({'id': f'I{len(incidents)+1}', 'subsystem': 'Power',
                'observation': f'Lowest reported voltage was {v:.2f} V.',
                'raw_time_s': t, 'window_s': [max(start, t-2), min(end, t+3)],
                'signals': [signal], 'severity': 'high'})
    # This flags sustained gaps between requested and measured speeds, not a failed shot.
    if points[TARGET] and points[ACTUAL]:
        shooter = sorted([(t, TARGET, v) for t, v in points[TARGET]]
                         + [(t, ACTUAL, v) for t, v in points[ACTUAL]])
        target = actual = None
        gap_start = None
        for t, name, value in shooter:
            if name == TARGET:
                target = value
            else:
                actual = value
            behind = target is not None and actual is not None and target >= 500 and target - actual >= 500
            if behind and gap_start is None:
                gap_start = t
            elif not behind and gap_start is not None:
                if t - gap_start >= 2:
                    incidents.append({'id': f'I{len(incidents)+1}', 'subsystem': 'Shooter',
                        'observation': f'The measured flywheel speed stayed at least 500 RPM below its requested speed for {t-gap_start:.1f} seconds.',
                        'raw_time_s': gap_start, 'window_s': [max(start, gap_start-2), min(end, t+2)],
                        'signals': [TARGET, ACTUAL], 'severity': 'medium'})
                gap_start = None
        if gap_start is not None and end - gap_start >= 2:
            incidents.append({'id': f'I{len(incidents)+1}', 'subsystem': 'Shooter',
                'observation': f'The measured flywheel speed stayed at least 500 RPM below its requested speed for at least {end-gap_start:.1f} seconds.',
                'raw_time_s': gap_start, 'window_s': [max(start, gap_start-2), end],
                'signals': [TARGET, ACTUAL], 'severity': 'medium'})
    slow_cycles = [(t, v) for t, v in points[CYCLE] if v > 40]
    if slow_cycles:
        t, v = max(slow_cycles, key=lambda item: item[1])
        incidents.append({'id': f'I{len(incidents)+1}', 'subsystem': 'Robot loop',
            'observation': f'The longest recorded robot loop was {v:.1f} ms; {len(slow_cycles)} recorded loops exceeded 40 ms.',
            'raw_time_s': t, 'window_s': [max(start, t-2), min(end, t+3)],
            'signals': [CYCLE], 'severity': 'medium'})
    incidents.sort(key=lambda incident: (incident['severity'] != 'high', incident['raw_time_s']))
    metrics = {'duration_s': round(end-start, 3),
               'lowest_battery_v': round(min(v for _, v in valid_volts), 2) if valid_volts else None,
               'confirmed_game_piece_actions': None,
               'auto_completion': None}
    metadata = {
        'event': state.get('/DriverStation/EventName'),
        'match_number': state.get('/DriverStation/MatchNumber'),
        'match_type': state.get('/DriverStation/MatchType'),
        'selected_auto': state.get('/RealOutputs/Match/SelectedAutoCommand')
            or state.get('/NetworkInputs/SmartDashboard/Auto Chooser'),
        'code_version': state.get('/Metadata/CodeVersion'),
    }
    return {'schema_version': 1, 'session_id': session_id, 'source_log': filename,
            'start_raw_s': start, 'end_raw_s': end, 'partial': partial,
            'boundary_note': 'Inferred from enable/disable records; gaps up to 30 seconds are grouped.',
            'metadata': metadata, 'timeline': timeline, 'metrics': metrics,
            'incidents': incidents, 'charts': points}


def ai_summary(review):
    key, model = os.getenv('OPENAI_API_KEY'), os.getenv('MATCH_REVIEW_MODEL')
    if not key or not model:
        return None
    # Only computed metrics, phase transitions and incident evidence leave the laptop.
    evidence = {k: review[k] for k in ('metrics', 'timeline', 'incidents', 'partial')}
    prompt = ('Summarize this robot session in JSON as {"sentences":[{"text":"...",'
              '"references":["I1"]}]}. Every sentence must reference at least one provided '
              'incident ID. State only supported observations; do not invent causes or scores. '
              'If no incident exists return an empty sentences array. Evidence: ' + json.dumps(evidence))
    payload = json.dumps({'model': model, 'input': prompt, 'store': False}).encode()
    req = urllib.request.Request('https://api.openai.com/v1/responses', data=payload,
          headers={'Authorization': 'Bearer '+key, 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=30) as response:
            body = json.load(response)
        texts = [part['text'] for item in body.get('output', []) if item.get('type') == 'message'
                 for part in item.get('content', []) if part.get('type') == 'output_text']
        parsed = json.loads(''.join(texts))
        valid = {incident['id'] for incident in review['incidents']}
        sentences = parsed['sentences']
        if not isinstance(sentences, list) or not sentences:
            return None
        for sentence in sentences:
            if (not isinstance(sentence.get('text'), str) or not sentence['text'].strip()
                or not isinstance(sentence.get('references'), list) or not sentence['references']
                or any(ref not in valid for ref in sentence['references'])):
                return None
        return sentences
    except (OSError, ValueError, KeyError, TypeError):
        return None


def render(review):
    return view.render(review, ai_summary(review), VOLTAGE, CYCLE, TARGET, ACTUAL)


def generate(path, out, ndjson=False):
    path = Path(path)
    rows = load(path)
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    digest = digest.hexdigest()[:12]
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for index, (start, end, partial) in enumerate(sessions(rows), 1):
        session_id = f'{digest}-{index}'
        review = analyze(rows, start, end, partial, session_id, path.name)
        (out / f'{session_id}.json').write_text(json.dumps(review, indent=2, allow_nan=False), encoding='utf-8')
        (out / f'{session_id}.html').write_text(render(review), encoding='utf-8')
        if ndjson:
            with (out / f'{session_id}.ndjson').open('w', encoding='utf-8') as f:
                for t, name, kind, value in rows:
                    if start <= t <= end and (not isinstance(value, float) or math.isfinite(value)):
                        bits = name.strip('/').split('/')
                        obj = {'session_id': session_id, 'raw_log_timestamp_s': t,
                               'match_relative_s': round(t-start, 6), 'subsystem': bits[-2] if len(bits)>1 else '',
                               'signal': name, 'value': value, 'units': UNITS.get(name)}
                        f.write(json.dumps(obj, allow_nan=False)+'\n')
        print(out / f'{session_id}.html')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', nargs='?', help='completed WPILOG')
    parser.add_argument('--output', default='match-reports')
    parser.add_argument('--ndjson', action='store_true')
    parser.add_argument('--watch', help='watch an import folder for stable WPILOGs')
    args = parser.parse_args()
    if args.watch:
        seen = {}
        while True:
            for path in Path(args.watch).glob('*.wpilog'):
                stat = path.stat()
                signature = (stat.st_size, stat.st_mtime_ns)
                if stat.st_size and time.time()-stat.st_mtime > 15 and seen.get(path) != signature:
                    try:
                        generate(path, args.output, args.ndjson)
                        seen[path] = signature
                    except (OSError, ValueError) as exc:
                        print(f'{path}: {exc}', flush=True)
            time.sleep(5)
    elif args.log:
        generate(args.log, args.output, args.ndjson)
    else:
        parser.error('provide a WPILOG or --watch')

if __name__ == '__main__':
    main()
