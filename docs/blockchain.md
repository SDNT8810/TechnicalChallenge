# Blockchain

We use Ganache as a local chain. Logger sends a self transaction with JSON in the data field. Extractor scans blocks and rebuilds a list of movement records.

Payload example:
```json
{"timestamp": 1234567890.0, "robot": {"time": 1234567890.0, "x": 1.23, "y": -0.45, "status": "moving"}}
```

Basic steps if you want to run pieces manually:
```bash
ganache-cli --deterministic --d ./blockchain_data
python Tools/blockchain_logger.py
python Tools/extract_blockchain_data.py
```

Result file: `records/decoded_blockchain_logs.json`.

That’s all we need for this project.
