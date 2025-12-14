# Quickstart Guide: Context-Based RAG Chatbot Integration

## Overview
This guide will help you set up and run the context-based RAG chatbot for the Physical AI Robotics Book. The system allows students to ask natural language questions about book content with responses grounded strictly in the book material.

## Prerequisites
- Python 3.11+ installed
- Node.js 18+ installed (for Docusaurus frontend)
- Access to OpenAI Agents SDK with Gemini model
- Neon PostgreSQL database account
- Qdrant (optional) for retrieval logs
- Git for version control

## Backend Setup (FastAPI)

### 1. Clone and Navigate to Backend
```bash
cd chatbot-backend
```

### 2. Create Virtual Environment and Install Dependencies
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Set Up Environment Variables
Create a `.env` file in the backend root:
```env
GEMINI_API_KEY=your_gemini_api_key
DATABASE_URL=your_neon_postgres_connection_string
QDRANT_URL=optional_qdrant_url
DOCS_PATH=../docs  # Path to your docs/ folder
MAX_CONTEXT_TOKENS=3000  # Maximum tokens for context
```

### 4. Run Database Migrations (if using database storage)
```bash
python -m alembic upgrade head
```

### 5. Start the Backend Server
```bash
uvicorn app.main:app --reload --port 8000
```

The backend will be available at `http://localhost:8000`.

## Frontend Integration (Docusaurus)

### 1. Install Chat Widget Dependencies
```bash
cd your-docusaurus-project
npm install --save react-markdown remark-gfm  # Additional dependencies if needed
```

### 2. Integrate Chat Widget Component
The chat widget can be added to your Docusaurus layout to appear on all pages:

```jsx
// In your Docusaurus theme or layout component
import ChatWidget from './src/components/ChatWidget/ChatWidget';

function Layout({children}) {
  return (
    <>
      <main>
        {children}
      </main>
      <ChatWidget />
    </>
  );
}
```

## API Usage

### Full Book Context Mode
To ask a question using the entire book context:

```bash
curl -X POST http://localhost:8000/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain how ROS 2 handles message passing between nodes"
  }'
```

### Selected Text Mode
To ask a question using only selected text:

```bash
curl -X POST http://localhost:8000/chat/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this mean?",
    "selected_text": "ROS 2 uses a publish-subscribe model for message passing between nodes..."
  }'
```

### Health Check
Verify the service is running:

```bash
curl http://localhost:8000/health
```

## Frontend Integration

### 1. Basic Widget Usage
```jsx
import ChatWidget from '../components/ChatWidget/ChatWidget';

function MyPage() {
  return (
    <div>
      <h1>My Content</h1>
      {/* Your page content */}
      <ChatWidget />
    </div>
  );
}
```

### 2. Text Selection Integration
The widget automatically detects selected text on the page and offers the option to ask questions about the selected content.

### 3. Mode Switching
Users can switch between:
- **Full Book Mode**: Questions answered from entire book content
- **Selected Text Mode**: Questions answered only from highlighted text

## Configuration Options

### Backend Configuration
- `MAX_CONTEXT_TOKENS`: Maximum tokens to send to the AI model
- `CONTEXT_EXTRACTION_STRATEGY`: Algorithm for extracting relevant context
- `RESPONSE_TIMEOUT`: Maximum time to wait for AI response
- `ENABLE_STREAMING`: Whether to stream responses to the frontend

### Frontend Configuration
- `DEFAULT_MODE`: Default chat mode (full-book or selected-text)
- `WIDGET_POSITION`: Position of the chat widget (bottom-right, etc.)
- `THEME`: Color theme options for the widget

## Testing the Integration

### 1. Unit Tests
```bash
cd chatbot-backend
python -m pytest tests/unit/
```

### 2. Integration Tests
```bash
cd chatbot-backend
python -m pytest tests/integration/
```

### 3. Manual Testing
1. Start both backend and Docusaurus frontend
2. Navigate to a book page
3. Ask a question about the content
4. Verify the response is grounded in book content
5. Test both full book and selected text modes

## Troubleshooting

### Common Issues

1. **"Context too large" errors**:
   - Solution: Adjust `MAX_CONTEXT_TOKENS` in your configuration

2. **API key authentication failures**:
   - Solution: Verify your GEMINI_API_KEY is correctly set in environment variables

3. **No response from AI service**:
   - Solution: Check network connectivity and API endpoint availability

4. **Text selection not working**:
   - Solution: Verify the text selection listener is properly attached to the document

### Performance Tips

1. **Optimize context extraction**: Use efficient algorithms to identify the most relevant book sections
2. **Implement caching**: Cache frequently requested information to improve response times
3. **Monitor token usage**: Keep track of context size to stay within model limits

## Next Steps

1. Implement conversation history persistence
2. Add support for follow-up questions
3. Enhance the text selection experience
4. Implement analytics for usage tracking
5. Add support for additional document formats