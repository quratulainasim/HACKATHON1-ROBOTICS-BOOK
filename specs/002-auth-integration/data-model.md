# Data Model: Authentication Integration

## User
**Description**: Represents a registered user in the authentication system
**Attributes**:
- id: String (UUID) - Unique identifier for the user
- email: String - User's email address (unique, validated)
- hashed_password: String - Securely hashed password (BCrypt/Argon2)
- name: String - User's display name
- created_at: DateTime - Account creation timestamp
- updated_at: DateTime - Last update timestamp
- email_verified: Boolean - Whether email has been verified
- email_verification_token: String - Token for email verification
- password_reset_token: String - Token for password reset
- password_reset_expires: DateTime - Expiration for password reset token
- avatar_url: String - URL to user's profile picture
- is_active: Boolean - Whether account is active/enabled
- last_login: DateTime - Timestamp of last successful login
- login_count: Number - Count of successful logins

## Session
**Description**: Represents an active user session with security tokens
**Attributes**:
- id: String (UUID) - Unique session identifier
- user_id: String (foreign key to User.id) - Associated user
- session_token: String - Secure session token (encrypted)
- refresh_token: String - Refresh token for session renewal
- expires_at: DateTime - Session expiration time
- created_at: DateTime - Session creation time
- last_accessed: DateTime - Last time session was used
- ip_address: String - IP address of session origin
- user_agent: String - Browser/device information
- is_active: Boolean - Whether session is currently valid
- device_info: JSON - Additional device information

## OAuthProvider
**Description**: Stores information about OAuth provider associations
**Attributes**:
- id: String (UUID) - Unique identifier
- user_id: String (foreign key to User.id) - Associated user
- provider: String (enum: google, github, microsoft) - OAuth provider name
- provider_user_id: String - User ID from the provider
- provider_access_token: String - Encrypted access token
- provider_refresh_token: String - Encrypted refresh token
- provider_expires_at: DateTime - Token expiration from provider
- created_at: DateTime - Association creation time
- scopes: Array<String> - Permissions granted by user

## Permission
**Description**: Defines specific permissions for role-based access control
**Attributes**:
- id: String (UUID) - Unique permission identifier
- name: String - Permission name (e.g., "read:content", "admin:users")
- description: String - Human-readable description of permission
- created_at: DateTime - Creation timestamp
- updated_at: DateTime - Last update timestamp

## Role
**Description**: Groups of permissions that can be assigned to users
**Attributes**:
- id: String (UUID) - Unique role identifier
- name: String - Role name (e.g., "admin", "editor", "viewer")
- description: String - Human-readable description of role
- permissions: Array<String> - List of permission IDs associated with role
- created_at: DateTime - Creation timestamp
- updated_at: DateTime - Last update timestamp

## UserRole
**Description**: Junction table linking users to roles
**Attributes**:
- id: String (UUID) - Unique identifier
- user_id: String (foreign key to User.id) - Associated user
- role_id: String (foreign key to Role.id) - Assigned role
- assigned_by: String (foreign key to User.id) - User who assigned role
- assigned_at: DateTime - Timestamp of role assignment
- expires_at: DateTime - Optional expiration date for role