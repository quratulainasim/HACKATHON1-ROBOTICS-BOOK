// docusaurus.webpack.config.js
// Webpack configuration for Docusaurus with proxy settings
module.exports = {
  devServer: {
    proxy: {
      '/api/auth': {
        target: process.env.AUTH_SERVER_URL || 'http://localhost:3001',
        changeOrigin: true,
        secure: false, // Set to true in production with proper SSL
        onProxyReq: (proxyReq, req, res) => {
          console.log(`Proxying ${req.method} ${req.url} to auth server`);
        },
        onProxyRes: (proxyRes, req, res) => {
          // Allow credentials to be passed through proxy
          proxyRes.headers['Access-Control-Allow-Origin'] = req.headers.origin || 'http://localhost:3000';
          proxyRes.headers['Access-Control-Allow-Credentials'] = 'true';
          proxyRes.headers['Access-Control-Allow-Headers'] = 'Origin, X-Requested-With, Content-Type, Accept, Authorization';
        },
      },
      '/api/session-info': {
        target: process.env.AUTH_SERVER_URL || 'http://localhost:3001',
        changeOrigin: true,
        secure: false,
      },
      '/api/logout-custom': {
        target: process.env.AUTH_SERVER_URL || 'http://localhost:3001',
        changeOrigin: true,
        secure: false,
      },
    },
  },
};