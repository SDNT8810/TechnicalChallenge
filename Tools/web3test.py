from web3 import Web3

# Connect to Ganache
try:
    w3 = Web3(Web3.HTTPProvider('http://localhost:8545'))
    if not w3.is_connected():
        raise Exception("Failed to connect to Ganache")
    else:
        # Check connection
        print(f"Connected: {w3.is_connected()}")

        # List accounts
        accounts = w3.eth.accounts
        print(f"Accounts: {accounts}")

        # Check balance
        balance = w3.eth.get_balance(accounts[0])
        print(f"Balance: {w3.from_wei(balance, 'ether')} ETH")

        # Get latest block
        block = w3.eth.get_block('latest')
        print(f"Latest block: {block.number}")

except Exception as e:
    print(f"Error connecting to Ganache: {e}")



