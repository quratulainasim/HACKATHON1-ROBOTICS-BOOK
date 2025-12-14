<!-- SYNC IMPACT REPORT:
Version change: 1.0.0 → 1.1.0
Modified principles: All principles updated to authentication focus
Added sections: Authentication-specific principles and architecture
Removed sections: Physical AI Robotics principles
Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending
  - .specify/templates/spec-template.md ⚠ pending
  - .specify/templates/tasks-template.md ⚠ pending
  - .specify/templates/commands/*.md ⚠ pending
  - README.md ⚠ pending
Follow-up TODOs: None
-->

# Authentication Integration Constitution

## Core Principles

### Secure Authentication Architecture
Authentication systems must be secure, minimal, and standards-based. All authentication logic runs on the Node.js server with no secrets exposed to the frontend. Better Auth serves as the core authentication framework with Neon PostgreSQL as the persistent user database.

### Frontend Security Boundary
The Docusaurus frontend must never directly handle authentication secrets or perform authentication logic. All auth operations must be proxied through the Node.js backend server which handles credential validation, session management, and token issuance.

### Better Auth Foundation
Better Auth is the designated authentication framework for all authentication needs. All authentication features must be implemented using Better Auth's standard patterns and interfaces to ensure consistency and maintainability.

### Neon PostgreSQL Persistence
Neon PostgreSQL serves as the authoritative user database. All user account data, session information, and authentication-related data must be stored in Neon PostgreSQL following secure schema design principles.

### Backend Mediation Layer
The Node.js server acts as a secure mediation layer between the frontend and authentication providers, handling login, signup, session management, and token operations while providing clean API endpoints for frontend consumption.

### Developer Experience Excellence
Authentication setup must be fully automated through CLI tools with no manual database configuration required. The system must work out-of-the-box with minimal setup steps for developers.

### Extensibility Foundation
The authentication system must be designed with future extensibility in mind, supporting OAuth providers, admin dashboards, and advanced security features like rate limiting and session expiry management.

### Deployment Portability
Authentication system must function identically across different deployment platforms (Vercel/Netlify for frontend, Railway/Render for backend) without requiring code modifications.

### Privacy-First Design
Authentication system must store only minimal user data necessary for operation, respecting user privacy and following data minimization principles.

### Predictable API Design
All authentication APIs must follow predictable REST patterns and consistent response structures to facilitate integration with any frontend framework or platform.

## Architecture Standards

Authentication architecture must follow a three-tier model: Docusaurus frontend consuming Node.js backend services which implement Better Auth with Neon PostgreSQL. Session management must be handled server-side with secure token transmission to the frontend. All database interactions must use parameterized queries to prevent injection attacks.

## Development Workflow

Authentication feature development follows: security review → implementation → penetration testing → deployment validation. All authentication code must undergo security-focused code review before merging. Session handling and token management must be validated through automated security testing.

## Governance

This constitution governs all authentication-related development for the Docusaurus + Node.js backend project. All authentication implementations must comply with these principles. Security amendments require security team approval and penetration testing validation.

**Version**: 1.1.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-11