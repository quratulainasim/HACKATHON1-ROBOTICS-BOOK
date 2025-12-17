import express from "express";
import bcrypt from "bcrypt";
import jwt from "jsonwebtoken";
import { Pool } from "pg";
import dotenv from "dotenv";

dotenv.config();

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false // For Neon free tier
  }
});

const JWT_SECRET = process.env.JWT_SECRET || process.env.AUTH_SECRET || "fallback-secret";

const router = express.Router();

// === CRITICAL FIX: Handle CORS preflight for all routes ===
router.options('*', (req, res) => {
  res.set('Access-Control-Allow-Origin', process.env.FRONTEND_URL || '*'); // Or your Vercel domain
  res.set('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
  res.set('Access-Control-Allow-Headers', 'Content-Type, Authorization');
  res.set('Access-Control-Allow-Credentials', 'true');
  res.status(204).end();
});
// ===============================================================

// JWT utility functions
const generateToken = (userId: string | number) => {
  return jwt.sign({ userId: String(userId) }, JWT_SECRET, { expiresIn: '7d' });
};

const verifyToken = (token: string) => {
  try {
    return jwt.verify(token, JWT_SECRET) as { userId: string };
  } catch (error) {
    return null;
  }
};

// Sign up endpoint
router.post('/sign-up/email', async (req, res) => {
  try {
    const { email, password, name } = req.body;
    if (!email || !password || !name) {
      return res.status(400).json({ error: 'Email, password, and name are required' });
    }

    const existingUser = await pool.query(
      'SELECT id FROM users WHERE email = $1',
      [email.toLowerCase()]
    );
    if (existingUser.rows.length > 0) {
      return res.status(400).json({ error: 'User already exists' });
    }

    if (password.length < 8) {
      return res.status(400).json({ error: 'Password must be at least 8 characters long' });
    }

    const hashedPassword = await bcrypt.hash(password, 10);

    const result = await pool.query(
      `INSERT INTO users (email, password_hash, name, email_verified, created_at, updated_at)
       VALUES ($1, $2, $3, $4, $5, $6)
       RETURNING id, email, name, created_at`,
      [email.toLowerCase(), hashedPassword, name, false, new Date(), new Date()]
    );

    const user = result.rows[0];
    const token = generateToken(user.id);

    return res.status(201).json({
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        createdAt: user.created_at
      },
      token
    });
  } catch (error) {
    console.error('Sign up error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});

// Sign in endpoint
router.post('/sign-in/email', async (req, res) => {
  try {
    const { email, password } = req.body;
    if (!email || !password) {
      return res.status(400).json({ error: 'Email and password are required' });
    }

    const result = await pool.query(
      'SELECT id, email, password_hash, name FROM users WHERE email = $1',
      [email.toLowerCase()]
    );

    if (result.rows.length === 0) {
      return res.status(401).json({ error: 'Invalid credentials' });
    }

    const user = result.rows[0];
    const isValid = await bcrypt.compare(password, user.password_hash);

    if (!isValid) {
      return res.status(401).json({ error: 'Invalid credentials' });
    }

    const token = generateToken(user.id);

    return res.json({
      user: {
        id: user.id,
        email: user.email,
        name: user.name
      },
      token
    });
  } catch (error) {
    console.error('Sign in error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});

// Middleware to verify JWT
const authenticateToken = (req: express.Request, res: express.Response, next: express.NextFunction) => {
  const authHeader = req.headers['authorization'];
  const token = authHeader && authHeader.split(' ')[1];

  if (!token) {
    return res.status(401).json({ error: 'Access token required' });
  }

  const decoded = verifyToken(token);
  if (!decoded) {
    return res.status(403).json({ error: 'Invalid or expired token' });
  }

  (req as any).userId = decoded.userId;
  return next();
};

// Get user profile
router.get('/user', authenticateToken, async (req, res) => {
  try {
    const userId = (req as any).userId;
    const result = await pool.query(
      'SELECT id, email, name, email_verified, created_at FROM users WHERE id = $1',
      [userId]
    );

    if (result.rows.length === 0) {
      return res.status(404).json({ error: 'User not found' });
    }

    const user = result.rows[0];

    return res.json({
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        emailVerified: user.email_verified,
        createdAt: user.created_at
      }
    });
  } catch (error) {
    console.error('Get user error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});

// Sign out
router.post('/sign-out', authenticateToken, (req, res) => {
  return res.json({ message: 'Signed out successfully' });
});

export default router;
