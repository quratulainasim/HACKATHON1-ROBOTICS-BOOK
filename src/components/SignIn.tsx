import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { Navigate } from '@docusaurus/router';
import authStyles from '../../src/pages/auth.module.css'; // Import the auth styles

interface SignInProps {
  onSwitchToSignUp?: () => void;
}

const SignInContent: React.FC<SignInProps> = ({ onSwitchToSignUp }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { signIn } = useAuth();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await signIn(email, password);
      // Redirect to main book content after successful sign in
      window.location.href = '/';
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Sign in failed');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div>
      {error && (
        <div className={authStyles.errorMessage}>
          {error}
        </div>
      )}
      <form onSubmit={handleSubmit} className={authStyles.authForm}>
        <div className={authStyles.inputGroup}>
          <label htmlFor="signin-email" className={authStyles.inputLabel}>
            Email Address
          </label>
          <div style={{ position: 'relative' }}>
            <i className={`fa fa-envelope ${authStyles.inputIcon}`}></i>
            <input
              type="email"
              id="signin-email"
              className={authStyles.inputField}
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              placeholder="Enter your email"
            />
          </div>
        </div>
        <div className={authStyles.inputGroup}>
          <label htmlFor="signin-password" className={authStyles.inputLabel}>
            Password
          </label>
          <div style={{ position: 'relative' }}>
            <i className={`fa fa-lock ${authStyles.inputIcon}`}></i>
            <input
              type="password"
              id="signin-password"
              className={authStyles.inputField}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              placeholder="Enter your password"
            />
          </div>
        </div>
        <button
          type="submit"
          className={authStyles.submitButton}
          disabled={loading}
        >
          {loading ? (
            <>
              <span className={authStyles.loadingSpinner}></span>
              Signing In...
            </>
          ) : (
            'Sign In'
          )}
        </button>
      </form>
      {onSwitchToSignUp && (
        <div className={authStyles.switchForm}>
          <p className={authStyles.switchFormText}>
            Don't have an account?
          </p>
          <button
            type="button"
            className={authStyles.switchFormButton}
            onClick={onSwitchToSignUp}
          >
            Sign up here
          </button>
        </div>
      )}
    </div>
  );
};

const SignIn: React.FC<SignInProps> = (props) => {
  return (
    <SignInContent {...props} />
  );
};

export default SignIn;