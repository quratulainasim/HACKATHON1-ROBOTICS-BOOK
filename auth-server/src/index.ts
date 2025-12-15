const express = require('express');
const cors = require('cors');
const dotenv = require('dotenv');
const { betterAuth } = require('better-auth');

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;

// FIXED CORS: Allow local + your live Vercel frontend
app.use(cors({
  origin: [
    'http://localhost:3000',  // Local Docusaurus dev
    'https://hackathon-1-robotics-book.vercel.app',  // Your main live URL
    'https://hackathon-1-robotics-book-oono8cq32.vercel.app',  // Preview URL if still active
    // Add any other Vercel branches/previews here
  ],
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization', 'Set-Cookie'],
}));

// Handle preflight requests explicitly (extra safety for Better Auth)
app.options('*', cors());

// Parse JSON
app.use(express.json());

// Initialize Better Auth
const auth = betterAuth({
  app: {
    name: 'Physical AI Robotics Auth',
    baseURL: process.env.AUTH_BASE_URL || `http://localhost:${PORT}`,
    siteURL: process.env.FRONTEND_URL || 'https://hackathon-1-robotics-book.vercel.app'
  },
  database: {
    url: process.env.DATABASE_URL,
    type: 'postgres'
  },
  socialProviders: {},
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false  // Set true if you want verification emails
  },
  secret: process.env.AUTH_SECRET || 'super-secret-fallback-for-dev-only'
});

// Mount Better Auth routes
app.use(auth.handler);

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'OK', timestamp: new Date().toISOString() });
});

// Start server
app.listen(PORT, () => {
  console.log(`Auth server running on port ${PORT}`);
  console.log(`Frontend allowed: localhost:3000 + https://hackathon-1-robotics-book.vercel.app`);
  console.log('CORS preflight fixed — sign in should work now!');
});
