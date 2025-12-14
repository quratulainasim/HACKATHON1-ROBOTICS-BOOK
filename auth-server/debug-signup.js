const express = require('express');
const bcrypt = require('bcrypt');
const jwt = require('jsonwebtoken');
const { Pool } = require('pg');
require('dotenv').config();

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false // For Neon free tier
  }
});

const JWT_SECRET = process.env.JWT_SECRET || process.env.AUTH_SECRET || "fallback-secret";

const app = express();
app.use(express.json());

// Test the complete sign-up flow step by step
app.post('/test-full-signup', async (req, res) => {
  try {
    console.log('Starting sign-up process...');
    const { email, password, name } = req.body;

    console.log('Input validation...');
    if (!email || !password || !name) {
      return res.status(400).json({ error: 'Email, password, and name are required' });
    }

    console.log('Checking if user exists...');
    // Check if user already exists
    const existingUser = await pool.query(
      'SELECT id FROM users WHERE email = $1',
      [email.toLowerCase()]
    );
    console.log('Existing user query result:', existingUser.rows);

    if (existingUser.rows.length > 0) {
      return res.status(400).json({ error: 'User already exists' });
    }

    console.log('Validating password...');
    // Validate password strength
    if (password.length < 8) {
      return res.status(400).json({ error: 'Password must be at least 8 characters long' });
    }

    console.log('Hashing password...');
    // Hash password
    const hashedPassword = await bcrypt.hash(password, 10);
    console.log('Password hashed successfully');

    console.log('Creating user in database...');
    // Create user
    const result = await pool.query(
      `INSERT INTO users (email, password_hash, name, email_verified, created_at, updated_at)
       VALUES ($1, $2, $3, $4, $5, $6)
       RETURNING id, email, name, created_at`,
      [email.toLowerCase(), hashedPassword, name, false, new Date(), new Date()]
    );
    console.log('User created result:', result.rows);

    const user = result.rows[0];
    console.log('User object:', user);

    console.log('Generating JWT token...');
    // Generate JWT
    const token = jwt.sign({ userId: String(user.id) }, JWT_SECRET, { expiresIn: '7d' });
    console.log('JWT token generated:', token.substring(0, 20) + '...');

    res.status(201).json({
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        createdAt: user.created_at
      },
      token
    });
  } catch (error) {
    console.error('Full sign-up error:', error);
    res.status(500).json({ error: 'Internal server error', details: error.message });
  }
});

const PORT = 3002;
app.listen(PORT, () => {
  console.log(`Test server running on http://localhost:${PORT}`);
});