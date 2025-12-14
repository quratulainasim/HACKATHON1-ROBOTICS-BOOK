// API configuration constants
export const API_BASE_URL = typeof window !== 'undefined'
  ? window.location.origin
  : 'http://localhost:3000';

export const AUTH_SERVER_URL = process.env.AUTH_SERVER_URL || 'http://localhost:3001';