import express from 'express';
import cors from 'cors';
import { BetterAuth } from 'better-auth';
import { Client } from 'pg';

const app = express();
app.use(cors());
app.use(express.json());

// Initialize PostgreSQL client (Neon)
const db = new Client({
  connectionString: process.env.DATABASE_URL,
});
db.connect().catch(err => {
  console.error('Failed to connect to DB:', err);
  process.exit(1);
});

// Initialize BetterAuth
const auth = new BetterAuth({
  apiKey: process.env.BETTER_AUTH_API_KEY!,
  db,
});

// Test route
app.get('/', (req, res) => {
  res.send('Server is running!');
});

// Example: Signup route
app.post('/signup', async (req, res) => {
  try {
    const { email, password } = req.body;
    const user = await auth.signup(email, password);
    res.json(user);
  } catch (err: any) {
    res.status(400).json({ error: err.message });
  }
});

// Example: Login route
app.post('/login', async (req, res) => {
  try {
    const { email, password } = req.body;
    const token = await auth.login(email, password);
    res.json({ token });
  } catch (err: any) {
    res.status(400).json({ error: err.message });
  }
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`Server running on port ${PORT}`));
