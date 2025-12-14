# Context-Based RAG Chatbot

This is a context-based Retrieval-Augmented Generation (RAG) chatbot that allows students to ask questions about the Physical AI Robotics Book content and receive answers based strictly on the book content.

## Prerequisites

- Python 3.8 or higher
- Google Gemini API key
- Node.js and npm (for frontend integration)

## Setup Instructions

### 1. Install Dependencies

```bash
cd chatbot-backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Copy the `.env.example` file to `.env` and update with your API keys:

```bash
cp .env.example .env
```

Then edit the `.env` file and add your:
- `GEMINI_API_KEY`: Your Google Gemini API key
- Update other configuration values as needed

### 3. Start the Server

```bash
python start_server.py
```

Or directly with uvicorn:
```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

The server will start at `http://localhost:8000`

## API Endpoints

- `POST /chat/query` - Ask a question using full book context
- `POST /chat/selected-text` - Ask a question using selected text context
- `POST /chat/query-stream` - Streaming response for full book context
- `POST /chat/selected-text-stream` - Streaming response for selected text context
- `GET /health` - Health check endpoint

## Frontend Integration

The chatbot frontend components are located in the main Docusaurus site at `/src/components/ChatWidget/`. These React components can be integrated into your Docusaurus pages.

## Usage

Once the server is running:

1. The chatbot will read content from the `docs/` folder
2. Questions asked through the API will be answered based on the book content
3. Two modes are supported:
   - Full book context mode: Answers based on entire book content
   - Selected text mode: Answers based only on highlighted text

## Troubleshooting

- If you get API errors, verify your `GEMINI_API_KEY` is correct
- Make sure the `docs/` folder contains the book content in Markdown format
- Check that the server is running on the correct port
- Ensure the frontend is configured to connect to the correct backend URL

## Stopping the Server

Press `Ctrl+C` to stop the server.