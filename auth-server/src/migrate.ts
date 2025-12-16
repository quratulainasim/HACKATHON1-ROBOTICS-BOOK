import { betterAuth } from "better-auth";

async function migrate() {
  const auth = betterAuth({
    secret: process.env.AUTH_SECRET!,
    baseURL: process.env.AUTH_BASE_URL!,
    database: {
      url: process.env.DATABASE_URL!,
    },
  });

  await auth.migrate();
  console.log("Migration completed");
}

migrate().catch((err) => {
  console.error("Migration failed:", err);
  process.exit(1);
});
