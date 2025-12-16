// src/migrate.ts
import { Client } from "pg";
import "dotenv/config";

const client = new Client({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false, // for Neon/remote Postgres
  },
});

async function runMigrations() {
  try {
    await client.connect();
    console.log("Connected to database ✅");

    // Example migration: create users table for BetterAuth
    await client.query(`
      CREATE TABLE IF NOT EXISTS users (
        id SERIAL PRIMARY KEY,
        email VARCHAR(255) UNIQUE NOT NULL,
        password_hash TEXT,
        created_at TIMESTAMP DEFAULT NOW()
      );
    `);
    console.log("Users table ensured ✅");

    // Example migration: create sessions table
    await client.query(`
      CREATE TABLE IF NOT EXISTS sessions (
        id SERIAL PRIMARY KEY,
        user_id INT REFERENCES users(id) ON DELETE CASCADE,
        token TEXT NOT NULL,
        expires_at TIMESTAMP NOT NULL,
        created_at TIMESTAMP DEFAULT NOW()
      );
    `);
    console.log("Sessions table ensured ✅");

    console.log("All migrations completed successfully 🎉");
  } catch (err) {
    console.error("Migration failed ❌", err);
  } finally {
    await client.end();
    console.log("Disconnected from database");
  }
}

runMigrations();
