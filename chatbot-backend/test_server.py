"""
Simple test script to verify the chatbot server can start
"""
import subprocess
import sys
import time
import requests
import threading

def test_server_startup():
    """Test that the server can start without errors"""
    print("Testing server startup...")

    # Start the server in a subprocess
    process = subprocess.Popen([
        sys.executable, "-c",
        """
import uvicorn
from app.main import app
uvicorn.run(app, host='0.0.0.0', port=8000)
        """
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Give the server a moment to start
    time.sleep(5)

    # Check if the process is still running
    if process.poll() is None:
        print("✓ Server started successfully")

        # Try to make a health check request
        try:
            response = requests.get("http://localhost:8000/health", timeout=5)
            if response.status_code == 200:
                print("✓ Health check passed")
            else:
                print(f"✗ Health check failed with status: {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"✗ Health check failed: {e}")

        # Terminate the server process
        process.terminate()
        process.wait()
        print("✓ Server stopped successfully")
        return True
    else:
        # Get error output
        _, stderr = process.communicate()
        print(f"✗ Server failed to start: {stderr.decode()}")
        return False

if __name__ == "__main__":
    print("Running server startup test...")
    success = test_server_startup()

    if success:
        print("\n✓ All tests passed! The chatbot server is ready to use.")
        print("\nTo start the server permanently, run:")
        print("  python start_server.py")
        print("\nThe server will be available at http://localhost:8000")
    else:
        print("\n✗ Tests failed. Please check the error messages above.")
        sys.exit(1)