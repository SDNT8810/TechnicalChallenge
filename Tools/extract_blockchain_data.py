#!/usr/bin/env python3

from web3 import Web3
import json
from datetime import datetime
import os

def extract_blockchain_data():
    """Extract and save robot movement data from blockchain"""
    try:
        # Connect to Ganache
        w3 = Web3(Web3.HTTPProvider('http://localhost:8545'))
        if not w3.is_connected():
            print("❌ Failed to connect to Ganache")
            return
        
        print("✅ Connected to Ganache blockchain!")
        
        # Get basic info
        latest_block = w3.eth.block_number
        account = w3.eth.accounts[0]
        
        print(f"📊 Extracting data from {latest_block + 1} blocks...")
        print()
        
        movements = []
        
        # Check all blocks for our transactions
        for block_num in range(0, latest_block + 1):
            try:
                block = w3.eth.get_block(block_num)
                
                for tx_hash in block.transactions:
                    try:
                        tx = w3.eth.get_transaction(tx_hash)
                        
                        # Check if this is our logging transaction
                        if (tx['from'].lower() == account.lower() and 
                            tx.get('to') and tx['to'].lower() == account.lower() and 
                            hasattr(tx, 'input') and tx.input and len(tx.input) > 10):
                            
                            # The data is already in bytes format, just decode it
                            try:
                                # Convert HexBytes to string directly
                                data_str = tx.input.decode('utf-8')
                                movement_data = json.loads(data_str)
                                
                                # Add block and transaction info
                                movement_entry = {
                                    'block_number': block_num,
                                    'transaction_hash': tx_hash.hex(),
                                    'timestamp': movement_data['timestamp'],
                                    'human_time': datetime.fromtimestamp(movement_data['timestamp']).strftime('%Y-%m-%d %H:%M:%S'),
                                    'robot': movement_data['robot'],
                                }
                                
                                movements.append(movement_entry)
                                
                                print(f"🔗 Block {block_num}: {tx_hash.hex()[:10]}...")
                                print(f"   📅 {movement_entry['human_time']}")
                                print(f"   🔵 robot: ({movement_data['robot']['x']}, {movement_data['robot']['y']})", {movement_data['robot']['status']})
                                print(f"   📦 transaction_data size (bytes): {len(json.dumps(movement_data).encode('utf-8'))}")
                                print()
                                
                            except Exception as decode_error:
                                print(f"Decode error for block {block_num}: {decode_error}")
                                continue
                                
                    except Exception as tx_error:
                        continue
                        
            except Exception as block_error:
                continue
        
        # Sort by timestamp
        movements.sort(key=lambda x: x['timestamp'])
        
        print(f"📈 Successfully extracted {len(movements)} movement records!")
        
        # Save to file
        file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'records', 'decoded_blockchain_logs.json')
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, 'w') as f:
            json.dump(movements, f, indent=2)

        print(f"💾 Decoded logs saved to: {file_path}")

        # Create a summary
        if movements:
            print("\n📋 MOVEMENT SUMMARY:")
            print("=" * 60)
            print(f"First record: {movements[0]['human_time']}")
            print(f"Last record:  {movements[-1]['human_time']}")
            print(f"Total records: {len(movements)}")
            
            # Show robot position changes
            first_pos = movements[0]
            last_pos = movements[-1]
            
            print(f"\n🔵 robot Journey:")
            print(f"   Start: ({first_pos['robot']['x']}, {first_pos['robot']['y']})")
            print(f"   End:   ({last_pos['robot']['x']}, {last_pos['robot']['y']})")
            
        return movements
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return []

if __name__ == '__main__':
    extract_blockchain_data()
