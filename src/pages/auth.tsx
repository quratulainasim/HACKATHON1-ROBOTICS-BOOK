import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../components/AuthContext';
import SignIn from '../components/SignIn';
import SignUp from '../components/SignUp';
import styles from './auth.module.css';

const AuthPageContent: React.FC = () => {
  const [isSignUp, setIsSignUp] = useState(false);

  return (
    <Layout title={isSignUp ? "Sign Up" : "Sign In"} description="Authentication page">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authCardContent}>
            <div className={styles.authHeader}>
              <h1 className={styles.authTitle}>
                {isSignUp ? 'Create Account' : 'Welcome Back'}
              </h1>
              <p className={styles.authSubtitle}>
                {isSignUp
                  ? 'Sign up to access the Physical AI Robotics Book content'
                  : 'Sign in to continue your robotics learning journey'}
              </p>
            </div>

            {isSignUp ? (
              <SignUp onSwitchToSignIn={() => setIsSignUp(false)} />
            ) : (
              <SignIn onSwitchToSignUp={() => setIsSignUp(true)} />
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
};

const AuthPage: React.FC = () => {
  return (
    <AuthProvider>
      <AuthPageContent />
    </AuthProvider>
  );
};

export default AuthPage;