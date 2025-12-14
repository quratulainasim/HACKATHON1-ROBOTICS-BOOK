#!/usr/bin/env python3
"""
Start script for the Context-Based RAG Chatbot
"""
import subprocess
import sys
import os

def install_requirements():
    """Install required packages from requirements.txt"""
    print("Installing required packages...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
    print("Requirements installed successfully!")

def check_env_vars():
    """Check if required environment variables are set"""
    required_vars = ['GEMINI_API_KEY']
    missing_vars = []

    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"Warning: The following environment variables are not set: {', '.join(missing_vars)}")
        print("Please set them in your .env file or environment before running the server.")
        return False

    return True

def start_server():
    """Start the FastAPI server"""
    print("Starting the Context-Based RAG Chatbot server...")
    print("Server will be available at http://localhost:8000")
    print("Press Ctrl+C to stop the server")

    try:
        # Run uvicorn to start the server
        subprocess.run([
            "uvicorn",
            "app.main:app",
            "--host", "0.0.0.0",
            "--port", "8000",
            "--reload"
        ])
    except KeyboardInterrupt:
        print("\nShutting down the server...")
    except Exception as e:
        print(f"Error starting server: {e}")

def main():
    """Main function to run the chatbot server"""
    print("Context-Based RAG Chatbot - Startup Script")
    print("=" * 50)

    # Check if requirements are installed
    try:
        import fastapi
        import openai
        import pydantic
    except ImportError:
        print("Required packages not found. Installing...")
        install_requirements()

    # Check environment variables
    if not check_env_vars():
        print("\nYou can still start the server, but AI functionality may be limited.")
        input("Press Enter to continue anyway, or Ctrl+C to exit... ")

    # Start the server
    start_server()

if __name__ == "__main__":
    main()