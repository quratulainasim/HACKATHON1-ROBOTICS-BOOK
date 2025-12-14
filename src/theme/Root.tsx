// src/theme/Root.tsx
import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// This is the top-level component that wraps the entire app
// We use it to inject configuration into the window object
const Root = ({ children }: { children: React.ReactNode }) => {
  const context = useDocusaurusContext();
  const { siteConfig } = context;

  // Inject config into window object during SSR and client-side
  React.useEffect(() => {
    const customFields: any = siteConfig.customFields || {};
    (window as any).__APP_CONFIG__ = {
      chatServerUrl: customFields.chatServerUrl || 'http://localhost:8001',
      authServerUrl: customFields.authServerUrl || 'https://hackathon1-robotics-book-production.up.railway.app',
    };
  }, [siteConfig]);

  return <>{children}</>;
};

export default Root;