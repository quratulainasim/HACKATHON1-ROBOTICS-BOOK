"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = __importDefault(require("express"));
const bcrypt_1 = __importDefault(require("bcrypt"));
const jsonwebtoken_1 = __importDefault(require("jsonwebtoken"));
const pg_1 = require("pg");
const dotenv_1 = __importDefault(require("dotenv"));
dotenv_1.default.config();
const pool = new pg_1.Pool({
    connectionString: process.env.DATABASE_URL,
    ssl: {
        rejectUnauthorized: false // For Neon free tier
    }
});
const JWT_SECRET = process.env.AUTH_SECRET || "fallback-secret";
// JWT utility functions
const generateToken = (userId) => {
    return jsonwebtoken_1.default.sign({ userId }, JWT_SECRET, { expiresIn: '7d' });
};
const verifyToken = (token) => {
    try {
        return jsonwebtoken_1.default.verify(token, JWT_SECRET);
    }
    catch (error) {
        return null;
    }
};
const authRouter = express_1.default.Router();
// Sign up endpoint
authRouter.post('/sign-up/email', async (req, res) => {
    try {
        const { email, password, name } = req.body;
        if (!email || !password || !name) {
            return res.status(400).json({ error: 'Email, password, and name are required' });
        }
        // Check if user already exists
        const existingUser = await pool.query('SELECT id FROM users WHERE email = $1', [email]);
        if (existingUser.rows.length > 0) {
            return res.status(400).json({ error: 'User already exists' });
        }
        // Hash password
        const hashedPassword = await bcrypt_1.default.hash(password, 10);
        // Create user
        const result = await pool.query(`INSERT INTO users (email, password_hash, name, email_verified)
       VALUES ($1, $2, $3, $4)
       RETURNING id, email, name, created_at`, [email, hashedPassword, name, false]);
        const user = result.rows[0];
        // Generate JWT
        const token = generateToken(user.id);
        res.status(201).json({
            user: {
                id: user.id,
                email: user.email,
                name: user.name,
                createdAt: user.created_at
            },
            token
        });
    }
    catch (error) {
        console.error('Sign up error:', error);
        res.status(500).json({ error: 'Internal server error' });
    }
});
// Sign in endpoint
authRouter.post('/sign-in/email', async (req, res) => {
    try {
        const { email, password } = req.body;
        if (!email || !password) {
            return res.status(400).json({ error: 'Email and password are required' });
        }
        // Find user
        const result = await pool.query('SELECT id, email, password_hash, name FROM users WHERE email = $1', [email]);
        if (result.rows.length === 0) {
            return res.status(401).json({ error: 'Invalid credentials' });
        }
        const user = result.rows[0];
        // Verify password
        const isValid = await bcrypt_1.default.compare(password, user.password_hash);
        if (!isValid) {
            return res.status(401).json({ error: 'Invalid credentials' });
        }
        // Generate JWT
        const token = generateToken(user.id);
        res.json({
            user: {
                id: user.id,
                email: user.email,
                name: user.name
            },
            token
        });
    }
    catch (error) {
        console.error('Sign in error:', error);
        res.status(500).json({ error: 'Internal server error' });
    }
});
// Middleware to verify JWT
const authenticateToken = (req, res, next) => {
    const authHeader = req.headers['authorization'];
    const token = authHeader && authHeader.split(' ')[1]; // Bearer TOKEN
    if (!token) {
        return res.status(401).json({ error: 'Access token required' });
    }
    const decoded = verifyToken(token);
    if (!decoded) {
        return res.status(403).json({ error: 'Invalid or expired token' });
    }
    req.userId = decoded.userId;
    next();
};
// Get user profile (protected route)
authRouter.get('/user', authenticateToken, async (req, res) => {
    try {
        const userId = req.userId;
        const result = await pool.query('SELECT id, email, name, email_verified, created_at FROM users WHERE id = $1', [userId]);
        if (result.rows.length === 0) {
            return res.status(404).json({ error: 'User not found' });
        }
        const user = result.rows[0];
        res.json({
            user: {
                id: user.id,
                email: user.email,
                name: user.name,
                emailVerified: user.email_verified,
                createdAt: user.created_at
            }
        });
    }
    catch (error) {
        console.error('Get user error:', error);
        res.status(500).json({ error: 'Internal server error' });
    }
});
// Sign out (client-side operation - just invalidate token on client)
authRouter.post('/sign-out', authenticateToken, (req, res) => {
    // In a real implementation, you might want to add the token to a blacklist
    res.json({ message: 'Signed out successfully' });
});
exports.default = authRouter;
//# sourceMappingURL=simple-auth.js.map