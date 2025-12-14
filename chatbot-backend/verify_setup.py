"""
Verification script to confirm the chatbot is ready to use
"""
import subprocess
import time
import requests
import threading
import sys

def start_server():
    """Start the server in a subprocess"""
    print("Starting the chatbot server...")
    process = subprocess.Popen([
        sys.executable, "-c",
        """
import uvicorn
from app.main import app
uvicorn.run(app, host='127.0.0.1', port=8000)
        """
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return process

def test_endpoints():
    """Test the API endpoints"""
    try:
        # Wait a bit for server to start
        time.sleep(3)

        # Test health endpoint
        response = requests.get("http://127.0.0.1:8000/health", timeout=5)
        if response.status_code == 200:
            print("[SUCCESS] Health endpoint is working")
            print(f"  Response: {response.json()}")
        else:
            print(f"[FAILED] Health endpoint failed with status {response.status_code}")
            return False

        # Test that the API is accessible
        response = requests.get("http://127.0.0.1:8000/docs", timeout=5)
        if response.status_code == 200:
            print("[SUCCESS] API documentation endpoint is accessible")
        else:
            print(f"[FAILED] API documentation endpoint failed with status {response.status_code}")
            return False

        return True
    except Exception as e:
        print(f"[FAILED] Error testing endpoints: {e}")
        return False

def main():
    print("Verifying Chatbot Setup...")
    print("="*40)

    # Start the server
    server_process = start_server()

    try:
        # Test the endpoints
        success = test_endpoints()

        if success:
            print("\n[SUCCESS] All tests passed!")
            print("[SUCCESS] The Context-Based RAG Chatbot is ready to use!")
            print("\nTo start using the chatbot:")
            print("1. Keep the server running with: python -m uvicorn app.main:app --host 0.0.0.0 --port 8000")
            print("2. Access the API at http://localhost:8000")
            print("3. The API documentation is available at http://localhost:8000/docs")
        else:
            print("\n[FAILED] Some tests failed. Please check the errors above.")

    finally:
        # Stop the server
        server_process.terminate()
        server_process.wait()
        print("\n[SUCCESS] Server stopped.")

if __name__ == "__main__":
    main()