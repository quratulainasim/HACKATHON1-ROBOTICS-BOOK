// Global configuration for the application
// This file is used to provide configuration to the client-side code

interface AppConfig {
  chatServerUrl: string;
  authServerUrl: string;
}

// Default configuration
const defaultConfig: AppConfig = {
  chatServerUrl: 'http://localhost:8001',
  authServerUrl: 'https://hackathon1-robotics-book-production.up.railway.app',
};

// Try to get configuration from window object (set by Docusaurus config or build process)
const getConfig = (): AppConfig => {
  // First try to get from global window object (injected by Docusaurus)
  if (typeof window !== 'undefined' && (window as any).__APP_CONFIG__) {
    return {
      chatServerUrl: (window as any).__APP_CONFIG__.chatServerUrl || defaultConfig.chatServerUrl,
      authServerUrl: (window as any).__APP_CONFIG__.authServerUrl || defaultConfig.authServerUrl,
    };
  }

  // For development, we can use environment variables during build time
  // But in Docusaurus, we'll rely on the injected config
  return defaultConfig;
};

// Function to get updated config (useful for components that need to access config after initial load)
export const getAppConfig = (): AppConfig => {
  return getConfig();
};

export const appConfig = getConfig();