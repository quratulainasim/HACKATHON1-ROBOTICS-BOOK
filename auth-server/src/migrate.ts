import { Client } from "pg";

const DATABASE_URL = process.env.DATABASE_URL;

if (!DATABASE_URL) {
  throw new Error("DATABASE_URL is not set");
}

async function migrate() {
  const client = new Client({
    connectionString: DATABASE_URL
  });

  await client.connect();

  /**
   * Better Auth schema (minimal required)
   * Safe to run multiple times
   */
  await client.query(`
    CREATE TABLE IF NOT EXISTS users (
      id TEXT PRIMARY KEY,
      email TEXT UNIQUE,
      name TEXT,
      image TEXT,
      created_at TIMESTAMP DEFAULT now()
    );
  `);

  await client.query(`
    CREATE TABLE IF NOT EXISTS sessions (
      id TEXT PRIMARY KEY,
      user_id TEXT REFERENCES users(id) ON DELETE CASCADE,
      expires_at TIMESTAMP NOT NULL
    );
  `);

  await client.end();
  console.log("Migration completed");
}

migrate().catch((err) => {
  console.error("Migration failed:", err);
  process.exit(1);
});
