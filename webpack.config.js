// webpack.config.js
const path = require('path');

module.exports = {
  resolve: {
    fallback: {
      "url": require.resolve("url/"),
      "util": require.resolve("util/"),
      "http": require.resolve("stream-http"),
      "https": require.resolve("https-browserify"),
      "stream": require.resolve("stream-browserify"),
      "assert": require.resolve("assert/"),
      "path": require.resolve("path-browserify"),
      "querystring": require.resolve("querystring-es3"),
      "zlib": require.resolve("browserify-zlib"),
    },
  },
  plugins: [
    new (require('webpack').ProvidePlugin)({
      Buffer: ['buffer', 'Buffer'],
    }),
  ],
};