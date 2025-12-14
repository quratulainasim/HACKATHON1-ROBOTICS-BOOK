# How to Run the Context-Based RAG Chatbot

## Complete Setup Guide

This guide will walk you through setting up and running the Context-Based RAG Chatbot for the Physical AI Robotics Book.

## Prerequisites

- Python 3.8 or higher installed
- Node.js and npm (for frontend integration)
- Google Gemini API key
- Git (to clone/download the repository)

## Step 1: Install Backend Dependencies

Navigate to the chatbot backend directory and install required packages:

```bash
cd chatbot-backend
pip install -r requirements.txt
```

## Step 2: Configure Environment Variables

The `.env` file is already configured with your credentials:

- `GEMINI_API_KEY`: Your Google Gemini API key (already set)
- `DATABASE_URL`: Neon PostgreSQL connection string (already set)
- `QDRANT_URL` and `QDRANT_API_KEY`: Qdrant vector database credentials (already set)

If you need to update these values, edit the `chatbot-backend/.env` file.

## Step 3: Prepare Book Content

Make sure your book content is in the `docs/` folder in Markdown format. The chatbot will read from this directory to answer questions.

## Step 4: Start the Backend Server

Run the following command to start the chatbot backend:

```bash
cd chatbot-backend
python start_server.py
```

Alternatively, you can run directly with uvicorn:

```bash
cd chatbot-backend
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

The server will start at `http://localhost:8000`

## Step 5: (Optional) Integrate with Frontend

The chatbot frontend components are already created in `src/components/ChatWidget/`. If you're using Docusaurus:

1. Make sure your Docusaurus site is set up
2. The ChatWidget component can be imported and used in your pages
3. The frontend connects to the backend at `http://localhost:8000` by default

## Step 6: Test the Chatbot

Once the server is running:

1. The API endpoints will be available:
   - `POST /chat/query` - Full book context mode
   - `POST /chat/selected-text` - Selected text mode
   - `GET /health` - Server health check

2. You can test the endpoints using curl or a tool like Postman:

```bash
# Test health endpoint
curl http://localhost:8000/health

# Test full book context query
curl -X POST http://localhost:8000/chat/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?", "user_id": "test-user"}'
```

## Step 7: Using the Chat Interface

If you've integrated the frontend components:

1. Visit your Docusaurus site
2. The chat widget should appear on pages
3. You can ask questions about the book content
4. Toggle between full book mode and selected text mode

## Troubleshooting

### Common Issues:

1. **Port already in use**: If port 8000 is already in use, change the port in the `.env` file and restart the server.

2. **API Key Issues**: Verify your `GEMINI_API_KEY` is valid and has the necessary permissions.

3. **Connection Issues**: Make sure the frontend is configured to connect to the correct backend URL.

4. **Docs Folder Missing**: Ensure your book content is in the `docs/` folder in Markdown format.

### Verifying Setup:

1. Check that the server starts without errors
2. Verify the health endpoint returns a 200 status
3. Test that the API endpoints respond to requests
4. Confirm the AI service can connect to the Gemini API

## Stopping the Server

To stop the server, press `Ctrl+C` in the terminal where it's running.

## Next Steps

1. Add your book content to the `docs/` folder
2. Customize the frontend components if needed
3. Test with real questions to ensure proper functionality
4. Monitor performance and adjust settings as needed

The chatbot is now ready to use! Students can ask questions about the Physical AI Robotics Book content and receive answers based strictly on the book's content.