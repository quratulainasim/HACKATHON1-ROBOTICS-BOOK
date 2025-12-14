// Custom server for proxying API requests during development
const express = require('express');
const path = require('path');
const { createProxyMiddleware } = require('http-proxy-middleware');

const app = express();
const PORT = process.env.PORT || 3000;
const AUTH_SERVER_URL = process.env.AUTH_SERVER_URL || 'http://localhost:3001';

// Proxy API requests to the auth server
app.use('/api/auth', createProxyMiddleware({
  target: AUTH_SERVER_URL,
  changeOrigin: true,
  pathRewrite: {
    '^/api/auth': '/api/auth', // Keep the path as is
  },
  onProxyReq: (proxyReq, req, res) => {
    console.log(`Proxying ${req.method} ${req.url} to ${AUTH_SERVER_URL}`);
  },
  onProxyRes: (proxyRes, req, res) => {
    // Ensure CORS headers allow credentials
    proxyRes.headers['Access-Control-Allow-Origin'] = req.headers.origin || 'http://localhost:3000';
    proxyRes.headers['Access-Control-Allow-Credentials'] = 'true';
    proxyRes.headers['Access-Control-Allow-Headers'] = 'Origin, X-Requested-With, Content-Type, Accept, Authorization';
  }
}));

// Serve static files from the built Docusaurus site
app.use(express.static(path.join(__dirname, '../build')));

// Handle all other routes by serving the Docusaurus index.html
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, '../build', 'index.html'));
});

app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
  console.log(`Proxying auth requests to: ${AUTH_SERVER_URL}`);
});