import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '../components/ChatWidget/ChatWidget';
import { AuthProvider } from '../components/AuthContext';
import type { Props } from '@theme/Layout';

const Layout = (props: Props): JSX.Element => {
  return (
    <AuthProvider>
      <OriginalLayout {...props} />
      <ChatWidget />
    </AuthProvider>
  );
};

export default Layout;