"""Presentation for standalone, pit-friendly reports."""
from html import escape as e


def graph(title, description, series, review, unit):
    available = [(label, color, name, review['charts'].get(name, [])) for label, color, name in series]
    available = [s for s in available if s[3]]
    if not available:
        return f'<article class="empty"><b>{e(title)}</b><br>These signals were not recorded.</article>'
    values = [v for _, _, _, points in available for _, v in points]
    low, high = min(values), max(values)
    padding = (high-low)*.1 or max(abs(high)*.05, 1)
    low, high = low-padding, high+padding
    start, end = review['start_raw_s'], review['end_raw_s']
    x = lambda t: 60 + 700*(t-start)/max(end-start, .001)
    y = lambda v: 20 + 160*(high-v)/(high-low)
    grid = ''.join(f'<line x1="60" y1="{20+i*40}" x2="760" y2="{20+i*40}" class="grid"/><text x="50" y="{24+i*40}" text-anchor="end">{high-i*(high-low)/4:.1f}</text>' for i in range(5))
    lines = ''
    for label, color, name, points in available:
        # Keep minima and maxima when reducing the graph so short spikes remain visible.
        stride = max(1, len(points)//350)
        reduced = []
        for pos in range(0, len(points), stride):
            bucket = points[pos:pos+stride]
            reduced += sorted({tuple(bucket[0]), tuple(bucket[-1]), tuple(min(bucket,key=lambda p:p[1])), tuple(max(bucket,key=lambda p:p[1]))})
        coords = ' '.join(f'{x(t):.1f},{y(v):.1f}' for t,v in reduced)
        lines += (f'<circle cx="{x(reduced[0][0]):.1f}" cy="{y(reduced[0][1]):.1f}" r="4" fill="{color}"/>' if len(reduced)==1 else f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="3"/>')
    names = {name for _,_,name,_ in available}
    markers = ''.join(f'<line class="marker" x1="{x(i["raw_time_s"]):.1f}" x2="{x(i["raw_time_s"]):.1f}" y1="20" y2="180"><title>{e(i["id"])}: {e(i["observation"])}</title></line>' for i in review['incidents'] if names.intersection(i['signals']))
    legend = ''.join(f'<span><i style="background:{color}"></i>{e(label)}</span>' for label,color,_,_ in available)
    return f'''<article class="chart"><div class="charthead"><div><h3>{e(title)}</h3><p>{e(description)}</p></div><b>{e(unit)}</b></div><div class="legend">{legend}<span><i class="flag"></i>Flagged moment</span></div><svg viewBox="0 0 780 215" role="img" aria-label="{e(title)} by elapsed time">{grid}{lines}{markers}<text x="60" y="207">Start · 0 s</text><text x="760" y="207" text-anchor="end">{end-start:.1f} s</text></svg><small>Time since first enable · vertical scale in {e(unit)}. Hover over a dashed line to see its finding.</small></article>'''


def render(review, ai, voltage, cycle, target, actual):
    issues, metrics = review['incidents'], review['metrics']
    if ai:
        recap = ''.join('<p>'+e(s['text'])+' <b>'+e(', '.join(s['references']))+'</b></p>' for s in ai)
        recap_label = 'AI recap · references checked'
    else:
        recap = f'<p>We found <b>{len(issues)} things to review</b> in the signals this tool checks. '+('Start with the cards below.' if issues else 'No checked rule was triggered.')+' Some information may not have been recorded, so this is not a complete health check.</p>'
        recap_label = 'Automated recap · works offline'
    battery = f"{metrics['lowest_battery_v']:.1f} V" if metrics['lowest_battery_v'] is not None else 'Not recorded'
    cards = ''
    for item in issues:
        t = item['raw_time_s']-review['start_raw_s']
        signals = ''.join(f'<code>{e(s)}</code>' for s in item['signals'])
        cards += f'''<article class="finding"><div class="findtop"><span class="tag {'urgent' if item['severity']=='high' else ''}">{'CHECK FIRST' if item['severity']=='high' else 'TAKE A LOOK'}</span><span>{e(item['id'])} · {e(item['subsystem'])}</span><b class="time">+{t:.1f} s</b></div><h3>{e(item['observation'])}</h3><p>Look at this moment in the original log to understand what happened around it.</p><details><summary>Where to look in AdvantageScope</summary><p>Open <b>{e(review['source_log'])}</b> and inspect raw time <b>{item['window_s'][0]:.3f}–{item['window_s'][1]:.3f} s</b>.</p>{signals}</details></article>'''
    if not cards:
        cards = '<div class="empty"><b>No issues triggered the current checks.</b><br>That does not mean every part of the robot was checked.</div>'
    timeline = ''.join(f'<div class="phase"><b>{e(x["mode"])}</b><small>+{x["elapsed_s"]:.1f} s · raw {x["raw_time_s"]:.3f} s</small></div>' for x in review['timeline']) or '<div class="empty">Robot mode was not recorded.</div>'
    battery_name = '/RealOutputs/Health/BatteryVolts' if review['charts'].get('/RealOutputs/Health/BatteryVolts') else voltage
    graphs = (graph('Battery voltage', 'A voltage drop may help explain why other systems struggled.', [('Battery','#2563eb',battery_name)], review,'volts')
            + graph('Shooter speed', 'Compare the speed requested with the speed measured.', [('Requested','#7c3aed',target),('Measured','#0d9488',actual)],review,'RPM')
            + graph('Robot loop time', 'A spike means a robot loop took unusually long to finish.', [('Loop time','#e46b2c',cycle)],review,'milliseconds'))
    meta = review['metadata']
    return f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Robot match review</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#f3f6fb;color:#15213a;font:15px/1.55 system-ui,-apple-system,Segoe UI,sans-serif}}main{{max-width:1100px;margin:auto;padding:30px 24px 70px}}.top{{display:flex;justify-content:space-between;margin-bottom:24px;color:#61718c}}.brand{{font-size:12px;letter-spacing:.13em;font-weight:800;color:#3459a9}}header{{background:linear-gradient(125deg,#142b59,#315ed0);color:white;border-radius:20px;padding:34px;box-shadow:0 16px 35px #233f7820}}.eyebrow{{font-size:11px;letter-spacing:.13em;font-weight:800;opacity:.8}}h1{{font-size:clamp(30px,4vw,46px);line-height:1.08;letter-spacing:-.035em;margin:9px 0}}header p{{color:#dbe7ff}}.pill{{display:inline-block;background:#ffffff20;border:1px solid #ffffff40;border-radius:999px;padding:5px 10px;font-size:12px;font-weight:700}}.stats{{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;margin:18px 0 30px}}.stat,.panel,.chart,.finding{{background:white;border:1px solid #e2e9f3;border-radius:15px;box-shadow:0 5px 18px #162c5108}}.stat{{padding:18px}}.stat small,.label{{display:block;color:#64748b;font-size:12px;font-weight:700}}.stat strong{{display:block;font-size:23px;margin:3px 0}}.stat span{{font-size:12px;color:#64748b}}.section{{margin:30px 0 14px}}h2{{font-size:22px;letter-spacing:-.025em;margin:0}}.section p,.chart p{{color:#64748b;margin:3px 0}}.panel{{padding:22px}}.recap{{border-left:4px solid #5b80e8}}.recap p{{font-size:16px;margin:8px 0 0}}.finding{{padding:20px;margin-bottom:12px}}.findtop{{display:flex;align-items:center;gap:10px;flex-wrap:wrap;color:#64748b;font-size:12px;font-weight:700}}.tag{{background:#fff7df;color:#8a6100;border-radius:6px;padding:4px 7px;font-size:10px;letter-spacing:.08em}}.urgent{{background:#fff0e9;color:#a63e1d}}.time{{margin-left:auto;color:#315aca;font-size:13px}}.finding h3{{font-size:16px;margin:14px 0 4px}}.finding p{{color:#52627b;margin:6px 0}}details{{margin-top:12px;border-top:1px solid #e2e9f3;padding-top:12px}}summary{{color:#315aca;font-weight:700;cursor:pointer}}code{{display:block;background:#f1f5fb;border-radius:6px;padding:6px 9px;margin:5px 0;overflow-wrap:anywhere;font-size:12px}}.timeline{{display:flex;gap:10px;flex-wrap:wrap}}.phase{{padding:12px 15px;background:#f5f8ff;border:1px solid #e1e9fa;border-radius:10px}}.phase b,.phase small{{display:block}}.phase small{{color:#64748b}}.chart{{padding:20px;margin-bottom:14px}}.charthead{{display:flex;justify-content:space-between}}.chart h3{{margin:0;font-size:17px}}.charthead b{{color:#4461a4;font-size:12px}}.legend{{display:flex;gap:20px;flex-wrap:wrap;margin-top:15px;font-size:12px;color:#52627b}}.legend i{{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}}.legend .flag{{width:16px;height:0;border-top:2px dashed #dc6b35;border-radius:0}}svg{{width:100%;height:auto}}svg text{{font-size:11px;fill:#71809a}}.grid{{stroke:#e8edf5}}.marker{{stroke:#dc6b35;stroke-width:2;stroke-dasharray:5 4}}.chart small,footer{{color:#64748b;font-size:12px}}.empty{{background:#f8faff;border:1px dashed #ccd7e9;border-radius:12px;padding:18px;color:#63728a;margin-bottom:14px}}.details{{display:grid;grid-template-columns:1fr 1fr;gap:14px}}.details b{{display:block;overflow-wrap:anywhere}}footer{{margin-top:30px}}@media(max-width:700px){{main{{padding:18px 14px 45px}}header{{padding:24px}}.stats{{grid-template-columns:1fr 1fr}}.details{{grid-template-columns:1fr}}.top small{{display:none}}}}
</style></head><body><main><div class="top"><div class="brand">TEAM 1403 / MATCH REVIEW</div><small>Made for a quick pit-side check</small></div><header><div class="eyebrow">YOUR ROBOT, AT A GLANCE</div><h1>Let's review the run.</h1><p>A plain-English look at the signals recorded during this session.</p><span class="pill">{'PARTIAL RECORDING' if review['partial'] else 'RECORDED SESSION'} · {metrics['duration_s']:.1f} seconds</span></header><div class="stats"><div class="stat"><small>THINGS TO REVIEW</small><strong>{len(issues)}</strong><span>From the checks available</span></div><div class="stat"><small>LOWEST BATTERY</small><strong>{battery}</strong><span>Positive readings only</span></div><div class="stat"><small>SESSION LENGTH</small><strong>{metrics['duration_s']:.1f} s</strong><span>From the first enable</span></div><div class="stat"><small>GAME PIECES</small><strong>Unknown</strong><span>No confirmed count available</span></div></div><div class="section"><h2>The quick read</h2><p>A starting point for the conversation.</p></div><section class="panel recap"><span class="label">{e(recap_label)}</span>{recap}</section><div class="section"><h2>What should we look at?</h2><p>Open a card for exact times and signal names.</p></div>{cards}<div class="section"><h2>How the run unfolded</h2><p>Time is measured from the first recorded enable.</p></div><section class="panel"><div class="timeline">{timeline}</div></section><div class="section"><h2>See the signals</h2><p>Use the graphs to spot changes over time.</p></div>{graphs}<div class="section"><h2>About this recording</h2><p>Keep the original log to investigate further.</p></div><section class="panel details"><div><span class="label">EVENT</span><b>{e(str(meta.get('event') or 'Not recorded'))}</b></div><div><span class="label">SELECTED AUTO</span><b>{e(str(meta.get('selected_auto') or 'Not recorded'))}</b></div><div><span class="label">SOURCE LOG</span><b>{e(review['source_log'])}</b></div><div><span class="label">SESSION ID</span><b>{e(review['session_id'])}</b></div></section><footer>Session boundaries are inferred from enable and disable records. This report only checks available signals; missing data is not proof that everything worked. Open the original WPILOG in AdvantageScope for the full picture.</footer></main></body></html>'''
