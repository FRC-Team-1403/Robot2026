# Laptop match review

The robot's AdvantageKit configuration and USB retrieval remain unchanged. Copy a **completed** `.wpilog` from the team's existing retrieval workflow to the laptop; keep the original file. Python 3.9 or newer is required; no packages are needed.

From the repository root, generate an offline HTML report and structured JSON:

```powershell
python tools/match_review/review.py path/to/match.wpilog --output match-reports --ndjson
```

Open the printed HTML path in a browser. `--ndjson` is optional and adds one Elasticsearch-compatible JSON object per line for supported primitive signal updates. Each object includes session ID, WPILOG timestamp in seconds, elapsed session time, subsystem, full signal path, value, and units when known. Import NDJSON using your Elasticsearch tooling; no Elastic service is needed to generate a report. The source WPILOG is never modified.

To automatically process files copied into a laptop folder, run the following and leave it running. It processes files after 15 seconds without modification; copy only completed logs to this folder. Restarting the watcher will regenerate the reports.

```powershell
python tools/match_review/review.py --watch path/to/imports --output match-reports --ndjson
```

The report gives an incident's raw timestamp and surrounding window in seconds. Open the original WPILOG in AdvantageScope, select the listed full signal paths, and navigate to that raw time window. Match-relative time is measured from the inferred first enable; it is not the FMS countdown. Sessions are inferred from enable/disable transitions, grouping gaps of at most 30 seconds; recordings that end while enabled are marked partial. A log without enable data is treated as partial. Check boundaries against the raw log, especially for multiple practice runs or unusual disable gaps.

The reader currently supports primitive boolean, double, float, int64, string, and JSON WPILOG values. Structured pose and array values are still available in AdvantageScope but are not analyzed or exported by this tool. Missing signals and unconfirmed actions are shown as unavailable. The voltage rule ignores zero readings, which may indicate initialization. The report is not a safety diagnosis.

For an optional AI recap, set `OPENAI_API_KEY` and `MATCH_REVIEW_MODEL` in the laptop environment before running the tool. Only derived metrics, timeline and incident evidence are sent to the OpenAI Responses API; the raw log is not uploaded. Each generated sentence must cite known incident IDs or it is discarded. Citation validation checks IDs, not whether a model's interpretation is correct; verify claims against the evidence. Without credentials, on network errors, or on invalid output, the report uses a labeled automated recap. The API may incur usage charges.

Run tool tests with:

```powershell
python -m unittest discover -s tools/match_review -p 'test_*.py'
```

The report checks recorded brownout and gyro-disconnect states, battery readings below 7 V, loops over 40 ms, and a shooter measured speed at least 500 RPM below a request of at least 500 RPM for two seconds. These are review thresholds, not proof of a failed mechanism or the cause of a problem. Graphs show requested and measured shooter speed together when both are available; dashed lines mark related findings. State values are carried forward between changes, as in AdvantageKit logs.
