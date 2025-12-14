# Quickstart Guide: Authentication Integration

## Getting Started

This guide will help you set up and run the authentication system for the Docusaurus + Node.js backend. The system uses Better Auth with Neon PostgreSQL for secure, scalable authentication.

### Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Neon PostgreSQL account and database
- Git for version control
- A Docusaurus project to integrate with

### Installation

1. **Clone the repository**:
```bash
git clone <your-repo-url>
cd <your-project-directory>
```

2. **Install backend dependencies**:
```bash
cd server
npm install
# or
yarn install
```

3. **Install frontend dependencies**:
```bash
cd ../frontend
npm install
# or
yarn install
```

4. **Set up environment variables**:
Create a `.env` file in the server directory with the following:
```env
DATABASE_URL=your_neon_postgresql_connection_string
AUTH_SECRET=your_secure_auth_secret
AUTH_URL=http://localhost:3001
NEXT_PUBLIC_SITE_URL=http://localhost:3000
```

### Configuration

1. **Database Setup**:
Run the database migrations to create the required tables:
```bash
npm run db:migrate
```

2. **Better Auth Configuration**:
The authentication system is configured in `server/config/auth.config.js`:
- Configure supported providers (email, OAuth)
- Set up email verification settings
- Define session management parameters
- Configure security settings (rate limiting, etc.)

3. **Neon PostgreSQL Connection**:
Ensure your Neon database connection is properly configured with:
- SSL enabled
- Proper connection pooling settings
- Appropriate user permissions

### Running the System

1. **Start the backend server**:
```bash
cd server
npm run dev
# Server will run on http://localhost:3001
```

2. **Start the Docusaurus frontend**:
```bash
cd frontend
npm run start
# Frontend will run on http://localhost:3000
```

3. **Access the application**:
- Frontend: http://localhost:3000
- Authentication endpoints: http://localhost:3001/api/auth/*
- Admin panel: http://localhost:3000/admin (when implemented)

### Basic Usage

#### User Registration
1. Navigate to `/register` on the frontend
2. Fill in registration form with valid information
3. Verify email address through the verification link sent to your email
4. User account is activated after verification

#### User Login
1. Navigate to `/login` on the frontend
2. Enter registered email and password
3. Authentication session is established
4. User gains access to protected content

#### Protected Routes
To protect routes in Docusaurus:
```javascript
// Use the authentication context
import { useAuth } from '../hooks/useAuth';

function ProtectedPage() {
  const { user, isAuthenticated, loading } = useAuth();

  if (loading) return <div>Loading...</div>;
  if (!isAuthenticated) return <div>Please log in</div>;

  return <div>Welcome, {user.name}!</div>;
}
```

### API Endpoints

The authentication system provides the following API endpoints:

- `POST /api/auth/register` - Create new user account
- `POST /api/auth/login` - Authenticate user with credentials
- `POST /api/auth/logout` - End current user session
- `GET /api/auth/me` - Get current user profile
- `POST /api/auth/forgot-password` - Initiate password reset
- `POST /api/auth/reset-password` - Complete password reset
- `GET /api/auth/verify-email` - Verify user's email address

### Development Workflow

1. **Backend Development**:
   - Run backend server with hot reload: `npm run dev`
   - All authentication logic resides in `/server/src/auth/`
   - API routes are defined in `/server/src/api/auth/`

2. **Frontend Integration**:
   - Authentication components in `/frontend/src/components/auth/`
   - Authentication hook in `/frontend/src/hooks/useAuth.js`
   - Protected routes implementation in `/frontend/src/pages/`

3. **Database Changes**:
   - Schema changes require migration files
   - Update data models in `/server/src/models/`
   - Run `npm run db:migrate` to apply changes

### Testing

1. **Unit Tests**:
```bash
# Run backend tests
cd server
npm run test

# Run frontend tests
cd frontend
npm run test
```

2. **End-to-End Tests**:
```bash
# Run e2e tests (requires running servers)
npm run test:e2e
```

### Troubleshooting

**Common Issues**:

1. **Database Connection Errors**:
   - Verify your Neon PostgreSQL connection string
   - Check firewall settings and IP allowlisting
   - Ensure SSL is properly configured

2. **Authentication Not Working**:
   - Check that AUTH_SECRET is consistent between frontend and backend
   - Verify CORS settings in the backend
   - Ensure the frontend is calling the correct API endpoints

3. **Session Not Persisting**:
   - Check cookie settings (HttpOnly, Secure, SameSite)
   - Verify that both frontend and backend are on compatible domains/ports
   - Ensure HTTPS is used in production

### Next Steps

1. Implement OAuth provider integration (Google, GitHub)
2. Add admin panel for user management
3. Implement role-based access control
4. Add audit logging for security monitoring
5. Set up production deployment configuration