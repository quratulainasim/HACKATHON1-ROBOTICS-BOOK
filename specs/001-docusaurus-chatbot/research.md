# Research: Context-Based RAG Chatbot Integration

## Decision: FastAPI Backend with OpenAI Agents SDK
**Rationale**: FastAPI provides excellent performance, automatic API documentation, and async support needed for AI service calls. OpenAI Agents SDK with Gemini provides the required AI capabilities for grounded responses.

**Alternatives considered**:
- Flask: Less performant, fewer built-in features
- Express.js: Would require different tech stack than specified
- Other AI SDKs: OpenAI Agents SDK specifically mentioned in requirements

## Decision: Deterministic Context Extraction Algorithm
**Rationale**: Since embeddings and semantic search are out of scope, we'll implement a deterministic algorithm that:
1. Maps the docs/ folder structure with chapters, headings, and sections
2. For full book mode: extracts content based on relevance to the query using keyword matching
3. For selected text mode: uses the exact selected text plus surrounding context (paragraph/siblings)

**Alternatives considered**:
- Vector embeddings: Explicitly out of scope
- Keyword search: Basic but effective for deterministic extraction
- Page/chapter based extraction: Good for maintaining context boundaries

## Decision: Frontend Integration Approach
**Rationale**: Using React components integrated into Docusaurus provides seamless user experience without leaving the book environment. The chat widget will overlay on pages with text selection capabilities.

**Alternatives considered**:
- Separate chat page: Would break reading flow
- Third-party chat widgets: Less customizable for specific requirements
- Native Docusaurus plugin: More complex to implement

## Decision: Context Size Management
**Rationale**: To respect model input limits while maintaining context quality, we'll implement:
1. Chunking algorithm that respects document structure (not just character limits)
2. Context prioritization based on query relevance
3. Fallback mechanisms when context exceeds limits

**Alternatives considered**:
- Fixed-size sliding windows: Might break document context
- First-N characters: Could cut off important context
- Smart summarization: More complex, potential for information loss

## Decision: Text Selection Mechanism
**Rationale**: Using browser Selection API to capture highlighted text and send to backend with coordinates/metadata to ensure exact text match.

**Alternatives considered**:
- Custom text selection UI: More complex to implement
- Right-click context menu: Less intuitive
- Browser native selection: Leverages existing user behavior

## Decision: Streaming Response Implementation
**Rationale**: Using Server-Sent Events (SSE) or WebSocket for streaming responses to provide better user experience during AI processing.

**Alternatives considered**:
- Polling: Less efficient
- Long-polling: More complex than SSE
- Simple POST/GET: No feedback during processing