const jwt = require('jsonwebtoken');
require('dotenv').config();

console.log('JWT_SECRET from env:', process.env.JWT_SECRET);
console.log('AUTH_SECRET from env:', process.env.AUTH_SECRET);

const JWT_SECRET = process.env.JWT_SECRET || process.env.AUTH_SECRET || "fallback-secret";
console.log('Using JWT_SECRET:', JWT_SECRET);

try {
  const token = jwt.sign({ userId: 'test-user-id' }, JWT_SECRET, { expiresIn: '7d' });
  console.log('JWT token generated successfully:', token.substring(0, 20) + '...');
} catch (error) {
  console.error('JWT token generation error:', error.message);
}