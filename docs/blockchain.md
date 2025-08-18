# Blockchain

Local chain: Ganache CLI (deterministic, persistent DB under `blockchain_data/`). Movement + status events are encoded as UTF‑8 JSON and sent in the `data` field of self transactions (from == to) with zero value. Extraction reconstructs an ordered log used for analysis and visualization.

## Payload Structure
```json
{"timestamp": 1234567890.0, "robot": {"time": 1234567890.0, "x": 1.23, "y": -0.45, "status": "moving"}}
```
Fields:
- `timestamp`: wall clock capture time.
- `robot`: nested object duplicating time plus planar pose (`x`,`y`) and textual status.

## Tool Scripts
| Script | Purpose |
|--------|---------|
| `blockchain_logger.py` | ROS 2 node: observes odometry + status, emits conditional blockchain or file log entries |
| `extract_blockchain_data.py` | Scans blocks, decodes self transactions, writes structured JSON log |
| `web3test.py` | Connectivity + environment sanity check (accounts, balance, latest block) |
| `vizualization.py` | Consumes decoded log + map + waypoints to create static path plot (not writing to chain) |

### Logger Implementation (`blockchain_logger.py`)
Flow:
1. Connects to `http://localhost:8545` via Web3. If this fails, switches to file-only logging.
2. Uses the first Ganache account as both sender and recipient (self transaction pattern simplifies filtering; zero ether transferred).
3. Subscribes to `/odom` (pose) and `/nav2_status` (string).
4. Timer (1 Hz) decides to record when either:
   - Linear displacement since last recorded point > 0.1 m, or
   - Status string changed.
5. Creates JSON: `{ timestamp, robot:{ time, x, y, status } }` rounding coordinates to 3 decimals.
6. Encodes JSON as hex (`0x` prefix) and estimates gas: `21000 + 16 * byte_count` (simple size-based heuristic) before sending.
7. On any send exception: falls back to appending the raw JSON (pretty) into `records/movement_log.json`.

### Extraction Implementation (`extract_blockchain_data.py`)
Steps:
1. Connect to Ganache; abort if unreachable.
2. Fetch latest block number; cache first account (same as logger account for filtering).
3. Iterate blocks 0..latest; for each transaction:
   - Include if `from == to == first_account` and `input` length > 10 (filters out empty calls).
4. Decode `tx.input` (strip `0x`, interpret hex as bytes, decode UTF‑8) then parse JSON.
5. Build record with: block_number, transaction_hash, timestamp, human_time (ISO), plus robot fields.
6. After scan: sort by `timestamp`; write `records/decoded_blockchain_logs.json` (overwrite) with indent=2.
7. Print summary: count, first/last timestamps, first/last positions.
Error handling: nested try/except blocks isolate malformed transactions or decode failures without stopping the full pass.

### Connectivity Test (`web3test.py`)
Minimal diagnostic script: checks connection, lists accounts, prints first account balance (ETH), and latest block number. Useful before investigating logger failures.

### Visualization Context (`vizualization.py`)
Reads the decoded blockchain log (or fallback file), waypoint CSVs, and map image/YAML. Rotates the map 90° clockwise for presentation, plots the path, annotates numbered waypoints, and marks start/end, saving `records/path_plot.png`.

## Manual Invocation
```bash
ganache-cli --deterministic --db ./blockchain_data
python Tools/blockchain_logger.py        # while simulation runs
python Tools/extract_blockchain_data.py  # after some movement
python Tools/vizualization.py            # optional plot
```

## Fallback Behavior
- Logger: on Web3 failure, data still persisted locally (`records/movement_log.json`).
- Extractor: skips decode errors; produces partial log instead of aborting.

## Potential Improvements
- Smart contract with events (indexed, cleaner filtering, future on-chain querying).
- Binary/CBOR compression to reduce transaction size and gas.
- Hash chaining or Merkle accumulation for tamper evidence of off-chain copies.
- Include waypoint index directly in payload.

## Output Artifacts
- `records/decoded_blockchain_logs.json`: canonical ordered telemetry (preferred).
- `records/movement_log.json`: local fallback (only if chain writes failed).
- `records/path_plot.png`: derived visualization.
