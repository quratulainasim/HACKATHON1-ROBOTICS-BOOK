// src/theme/Root.tsx
import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { AuthProvider } from '@site/src/components/AuthContext';  // ← Correct path for src/components/AuthContext.tsx

const Root = ({ children }: { children: React.ReactNode }) => {
  const context = useDocusaurusContext();
  const { siteConfig } = context;

  // Keep injecting config into window (this makes your chat work)
  React.useEffect(() => {
    const customFields: any = siteConfig.customFields || {};
    (window as any).__APP_CONFIG__ = {
      chatServerUrl: customFields.chatServerUrl || 'https://hackathon1-robotics-book-production-a9bd.up.railway.app',
      authServerUrl: customFields.authServerUrl || 'https://hackathon1-robotics-book-production-a9bd.up.railway.app',
    };
  }, [siteConfig]);

  // Wrap the entire app with AuthProvider — this fixes authentication!
  return <AuthProvider>{children}</AuthProvider>;
};

export default Root;
