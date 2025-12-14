const express = require('express');
const cors = require('cors');
const dotenv = require('dotenv');
const { betterAuth } = require('better-auth');

// Load environment variables
dotenv.config();

// Initialize Express app
const app = express();
const PORT = process.env.PORT || 3001;

// Middleware
app.use(cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3000',
  credentials: true
}));
app.use(express.json());

// Initialize Better Auth
const auth = betterAuth({
  app: {
    name: 'Physical AI Robotics Auth',
    baseURL: process.env.AUTH_BASE_URL || 'http://localhost:3001',
    siteURL: process.env.FRONTEND_URL || 'http://localhost:3000'
  },
  database: {
    url: process.env.DATABASE_URL,
    type: 'postgres'
  },
  socialProviders: {
    // Will add Google, GitHub, etc. later
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true
  },
  secret: process.env.AUTH_SECRET
});

// Mount Better Auth routes
app.use(auth);

// Additional API routes
app.get('/api/auth/session', (req, res) => {
  // This would return session info based on auth headers
  res.json({
    authenticated: !!req.headers.authorization,
    user: req.headers.authorization ? { id: 'temp_user', email: 'temp@example.com' } : null
  });
});

app.post('/api/auth/logout', (req, res) => {
  // Better Auth handles logout via middleware
  res.json({ success: true, message: 'Logged out successfully' });
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'OK', timestamp: new Date().toISOString() });
});

// Start server
app.listen(PORT, () => {
  console.log(`Authentication server running on port ${PORT}`);
  console.log(`Base URL: http://${process.env.HOST || 'localhost'}:${PORT}`);
});