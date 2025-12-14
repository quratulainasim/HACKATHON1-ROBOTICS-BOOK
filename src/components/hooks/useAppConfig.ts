import { useState, useEffect } from 'react';
import { getAppConfig } from '../../config';

// Custom hook to access app configuration with reactivity
export const useAppConfig = () => {
  const [config, setConfig] = useState(getAppConfig());

  useEffect(() => {
    // Update config when window.__APP_CONFIG__ becomes available
    // This handles the case where the config is injected after initial render
    const updateConfig = () => {
      setConfig(getAppConfig());
    };

    // Listen for config changes
    updateConfig();

    // If needed, you can add event listeners here for dynamic config updates
    // For now, we'll just update once when the component mounts
  }, []);

  return config;
};