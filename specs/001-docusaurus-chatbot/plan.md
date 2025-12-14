# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a context-based RAG chatbot for the Physical AI Robotics Book that allows students to ask natural language questions about book content. The system uses FastAPI backend with OpenAI Agents SDK (Gemini) to provide grounded responses based strictly on book content from the docs/ folder, supporting both full book context mode and selected text mode. The frontend features a professional chat widget integrated into Docusaurus pages with text selection capabilities. Based on research, we'll use deterministic context extraction algorithms, React-based frontend integration, and streaming responses for optimal user experience.

## Technical Context

**Language/Version**: Python 3.11 (for FastAPI backend), TypeScript/JavaScript (for Docusaurus frontend integration)
**Primary Dependencies**: FastAPI, OpenAI Agents SDK (Gemini), Neon PostgreSQL, Qdrant (optional), Docusaurus
**Storage**: Neon PostgreSQL for metadata and session logs, Qdrant for optional retrieval logs, Docusaurus docs/ folder for book content
**Testing**: pytest for backend testing, Jest for frontend components, manual testing for AI response quality
**Target Platform**: Linux server (backend), Web browser (frontend)
**Project Type**: Web application with backend API service and frontend integration
**Performance Goals**: <5 second response time for queries, handle 100+ concurrent users, respect model input size limits
**Constraints**: No external knowledge sources (book content only), no hallucination, deterministic context extraction, educational tone maintenance
**Scale/Scope**: Support for Physical AI Robotics Book content (~100-200 pages), 1000+ concurrent users, 99% availability during peak hours

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- Deterministic Context Extraction: System must extract content from docs/ folder using deterministic rules without embeddings or similarity search
- Grounded Responses Only: AI responses must be strictly based on provided context without hallucination
- Educational Tone Maintenance: Responses must maintain appropriate educational tone for robotics content
- No External Knowledge: System must not access external sources beyond the book content
- API Security: Backend API endpoints must be properly secured and validated
- Frontend Integration: Chat widget must integrate seamlessly with Docusaurus pages
- Performance Requirements: System must meet response time and concurrency goals
- Out-of-Scope Respect: No authentication, embeddings, or semantic similarity features as specified

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code Structure

```text
chatbot-backend/
├── app/
│   ├── main.py              # FastAPI application entrypoint
│   ├── api/
│   │   ├── chat.py          # Chat API endpoints
│   │   └── health.py        # Health check endpoint
│   ├── models/
│   │   ├── request.py       # Request models
│   │   └── response.py      # Response models
│   ├── services/
│   │   ├── context_extractor.py  # Context extraction logic
│   │   ├── chat_service.py       # Core chat service
│   │   └── ai_service.py         # AI interaction service
│   ├── utils/
│   │   ├── content_reader.py     # Reads docs/ folder content
│   │   └── text_processor.py     # Text processing utilities
│   └── config/
│       └── settings.py           # Configuration settings
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
└── requirements.txt

src/
├── components/
│   ├── ChatWidget/
│   │   ├── ChatWidget.tsx       # Main chat widget component
│   │   ├── ChatWindow.tsx        # Chat window UI
│   │   ├── Message.tsx           # Individual message component
│   │   └── InputArea.tsx         # Input area with text selection
│   └── hooks/
│       └── useChat.tsx           # Chat hook for state management
├── pages/
│   └── ChatPage.tsx              # Optional dedicated chat page
└── types/
    └── chat.d.ts                 # TypeScript definitions for chat
```

**Structure Decision**: Web application with separate backend API service (FastAPI) and frontend integration (Docusaurus React components). The backend handles AI interactions and context extraction while the frontend provides the chat interface integrated into Docusaurus pages.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
