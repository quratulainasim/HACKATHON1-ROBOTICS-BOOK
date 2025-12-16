import express from "express";
import { betterAuth } from "better-auth";
import { expressAdapter } from "@better-auth/node";

const app = express();
app.use(express.json());

const auth = betterAuth({
  database: {
    type: "postgres",
    url: process.env.DATABASE_URL!
  },
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!
});

app.use("/auth", expressAdapter(auth));

const port = process.env.PORT || 3000;
app.listen(port, () => {
  console.log(`Auth server running on port ${port}`);
});
