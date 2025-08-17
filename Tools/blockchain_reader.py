#!/usr/bin/env python3

from web3 import Web3
import json

def read_blockchain_logs():
    """Read and decode robot movement logs from the blockchain"""
    try:
        # Connect to Ganache
        w3 = Web3(Web3.HTTPProvider('http://localhost:8545'))
        if not w3.is_connected():
            print("❌ Failed to connect to Ganache")
            return
        
        print("✅ Connected to Ganache blockchain!")
        print(f"Latest block: {w3.eth.block_number}")
        
        # Get the account used for logging
        account = w3.eth.accounts[0]
        print(f"Checking transactions for account: {account}")
        
        # Get recent transactions
        latest_block = w3.eth.block_number
        movements = []
        
        print("\n📊 ROBOT MOVEMENT BLOCKCHAIN LOGS")
        print("=" * 60)
        
        # Check ALL blocks for transactions (from genesis to latest)
        print(f"Checking blocks 0 to {latest_block}...")
        
        for block_num in range(0, latest_block + 1):
            try:
                block = w3.eth.get_block(block_num, full_transactions=True)
                
                if len(block.transactions) > 0:
                    print(f"Block {block_num}: {len(block.transactions)} transactions")
                
                for tx_idx, tx in enumerate(block.transactions):
                    print(f"  TX {tx_idx}: from={tx['from'][:10]}... to={tx.get('to', 'None')[:10] if tx.get('to') else 'None'}... data_len={len(tx['data'])}")
                    
                    # Check if this is our logging transaction
                    if (tx['from'].lower() == account.lower() and 
                        tx.get('to') and tx['to'].lower() == account.lower() and 
                        tx['data'] != '0x' and len(tx['data']) > 10):
                        
                        try:
                            # Decode the data
                            data_hex = tx['data'][2:]  # Remove '0x'
                            data_bytes = bytes.fromhex(data_hex)
                            data_str = data_bytes.decode('utf-8')
                            movement_data = json.loads(data_str)
                            
                            movements.append({
                                'block': block_num,
                                'tx_hash': tx['hash'].hex(),
                                'data': movement_data
                            })
                            
                            print(f"🔗 Block {block_num}: {tx['hash'].hex()[:10]}...")
                            print(f"   📅 Time: {movement_data['timestamp']}")
                            print(f"   🔴 Robot1: ({movement_data['robot1']['x']}, {movement_data['robot1']['y']})")
                            print(f"   🔵 Robot2: ({movement_data['robot2']['x']}, {movement_data['robot2']['y']})")
                            print()
                            
                        except Exception as e:
                            print(f"   ⚠️  Failed to decode transaction data in block {block_num}: {e}")
                            print(f"   Raw data: {tx['data'][:100]}...")
            except Exception as e:
                print(f"Error reading block {block_num}: {e}")
                continue
        
        print(f"📈 Total movement logs found: {len(movements)}")
        
        # Save decoded logs to file for analysis
        with open('records/decoded_blockchain_logs.json', 'w') as f:
            json.dump(movements, f, indent=2)

        print(f"💾 Decoded logs saved to: records/decoded_blockchain_logs.json")

        return movements
        
    except Exception as e:
        print(f"❌ Error reading blockchain: {e}")
        return []

if __name__ == '__main__':
    read_blockchain_logs()
