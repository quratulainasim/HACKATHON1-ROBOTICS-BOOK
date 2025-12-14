const { Pool } = require('pg');
const bcrypt = require('bcrypt');
require('dotenv').config();

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

async function testUserCreation() {
  try {
    console.log('Testing user creation...');

    const email = 'test@example.com';
    const password = 'password123';
    const name = 'Test User';

    // Hash password
    const hashedPassword = await bcrypt.hash(password, 10);
    console.log('Password hashed successfully');

    // Create user
    const result = await pool.query(
      `INSERT INTO users (email, password_hash, name, email_verified, created_at, updated_at)
       VALUES ($1, $2, $3, $4, $5, $6)
       RETURNING id, email, name, created_at`,
      [email.toLowerCase(), hashedPassword, name, false, new Date(), new Date()]
    );

    console.log('User created successfully:', result.rows[0]);
  } catch (err) {
    console.error('User creation error:', err.message);
    console.error('Error code:', err.code);
    console.error('Error detail:', err.detail);
  } finally {
    await pool.end();
  }
}

testUserCreation();