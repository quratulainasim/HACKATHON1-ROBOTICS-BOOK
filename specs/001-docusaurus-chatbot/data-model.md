# Data Model: Context-Based RAG Chatbot Integration

## Entities

### Question
- **id**: string (UUID) - Unique identifier for the question
- **content**: string - The natural language question text
- **mode**: enum (full-book, selected-text) - The context mode used
- **selected_text**: string (optional) - Text that was selected in selected-text mode
- **timestamp**: datetime - When the question was submitted
- **user_id**: string (optional) - Identifier for the user session

### Context
- **id**: string (UUID) - Unique identifier for the context
- **source_type**: enum (full-book, selected-text, chapter, section) - Type of source
- **content**: string - The extracted context content
- **source_path**: string - Path to the source document in docs/ folder
- **relevance_score**: number - Relevance score to the question (0-1)
- **metadata**: object - Additional metadata about the context source

### Response
- **id**: string (UUID) - Unique identifier for the response
- **question_id**: string - Reference to the original question
- **content**: string - The AI-generated response content
- **confidence**: number - Confidence level in the response (0-1)
- **groundedness_score**: number - How well the response is grounded in context (0-1)
- **timestamp**: datetime - When the response was generated
- **mode**: enum (full-book, selected-text) - The mode used for this response

### ChatSession
- **id**: string (UUID) - Unique identifier for the session
- **user_id**: string (optional) - Identifier for the user
- **start_time**: datetime - When the session started
- **end_time**: datetime (optional) - When the session ended
- **mode**: enum (full-book, selected-text) - Current mode of the session
- **history**: array of objects - Conversation history with question/response pairs

### BookContent
- **id**: string (UUID) - Unique identifier for the content block
- **path**: string - File path in docs/ folder
- **title**: string - Title of the content (from heading)
- **content**: string - The actual content text
- **headings**: array of strings - Headings within this content block
- **tags**: array of strings - Tags/keywords associated with content
- **last_modified**: datetime - When the source file was last modified

## Relationships

- **ChatSession** contains many **Question** entities
- **Question** has one **Response** (1:1 relationship)
- **Question** uses **Context** to generate **Response**
- **Context** references **BookContent** as source
- **BookContent** may be referenced by multiple **Context** entities

## Validation Rules

### Question Validation
- Content must be between 5 and 500 characters
- Mode must be either "full-book" or "selected-text"
- If mode is "selected-text", selected_text must be provided

### Context Validation
- Content must be between 10 and 80% of model token limit
- Source_path must exist in docs/ folder
- Relevance_score must be between 0 and 1

### Response Validation
- Content must not exceed model token limits
- Groundedness_score must be calculated based on source attribution
- Must reference the original question_id

### ChatSession Validation
- Cannot have overlapping active sessions for same user
- Mode must be consistent throughout session unless explicitly changed
- History must maintain chronological order

## State Transitions

### ChatSession States
1. **Active** - New session created, awaiting first question
2. **Processing** - Question received, waiting for AI response
3. **Responded** - Response generated, ready for next question
4. **Ended** - Session terminated by user or timeout

### Mode Transitions
- Sessions can transition between "full-book" and "selected-text" modes
- Each question within a session can specify its own mode regardless of session default