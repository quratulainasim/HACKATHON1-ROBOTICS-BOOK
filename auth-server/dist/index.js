"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = __importDefault(require("express"));
const cors_1 = __importDefault(require("cors"));
const dotenv_1 = __importDefault(require("dotenv"));
const auth_routes_1 = __importDefault(require("./auth-routes"));
dotenv_1.default.config();
const app = (0, express_1.default)();
const PORT = process.env.PORT || "3001";
// CORS — allow your React app
app.use((0, cors_1.default)({
    origin: ["http://localhost:3000", "http://127.0.0.1:3000"],
    credentials: true,
}));
// Parse JSON
app.use(express_1.default.json());
// Mount auth routes at /api/auth
app.use("/api/auth", auth_routes_1.default);
// Optional: health check
app.get("/health", (_, res) => res.json({ status: "OK" }));
app.listen(PORT, () => {
    console.log(`Auth server running on http://localhost:${PORT}`);
    console.log(`Sign-up endpoint → http://localhost:${PORT}/api/auth/sign-up/email`);
    console.log(`Sign-in endpoint → http://localhost:${PORT}/api/auth/sign-in/email`);
});
//# sourceMappingURL=index.js.map