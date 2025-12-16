import { Client } from 'pg';

async function migrate() {
  const db = new Client({
    connectionString: process.env.DATABASE_URL,
  });

  try {
    await db.connect();
    console.log('Connected to DB for migration.');

    // Example migration: Create users table
    await db.query(`
      CREATE TABLE IF NOT EXISTS users (
        id SERIAL PRIMARY KEY,
        email TEXT UNIQUE NOT NULL,
        password TEXT NOT NULL,
        created_at TIMESTAMP DEFAULT NOW()
      )
    `);

    console.log('Migration completed.');
  } catch (err) {
    console.error('Migration failed:', err);
  } finally {
    await db.end();
  }
}

migrate();
