import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { Navigate } from '@docusaurus/router';
import authStyles from '../../src/pages/auth.module.css'; // Import the auth styles

interface SignUpProps {
  onSwitchToSignIn?: () => void;
}

const SignUpContent: React.FC<SignUpProps> = ({ onSwitchToSignIn }) => {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { signUp } = useAuth();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters long');
      return;
    }

    setLoading(true);

    try {
      await signUp(email, password, name);
      // Redirect to main book content after successful sign up
      window.location.href = '/';
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Sign up failed');
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
          <label htmlFor="signup-name" className={authStyles.inputLabel}>
            Full Name
          </label>
          <div style={{ position: 'relative' }}>
            <i className={`fa fa-user ${authStyles.inputIcon}`}></i>
            <input
              type="text"
              id="signup-name"
              className={authStyles.inputField}
              value={name}
              onChange={(e) => setName(e.target.value)}
              required
              placeholder="Enter your full name"
            />
          </div>
        </div>
        <div className={authStyles.inputGroup}>
          <label htmlFor="signup-email" className={authStyles.inputLabel}>
            Email Address
          </label>
          <div style={{ position: 'relative' }}>
            <i className={`fa fa-envelope ${authStyles.inputIcon}`}></i>
            <input
              type="email"
              id="signup-email"
              className={authStyles.inputField}
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              placeholder="Enter your email"
            />
          </div>
        </div>
        <div className={authStyles.inputGroup}>
          <label htmlFor="signup-password" className={authStyles.inputLabel}>
            Password
          </label>
          <div style={{ position: 'relative' }}>
            <i className={`fa fa-lock ${authStyles.inputIcon}`}></i>
            <input
              type="password"
              id="signup-password"
              className={authStyles.inputField}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              placeholder="Enter your password (min 8 characters)"
            />
          </div>
        </div>
        <div className={authStyles.inputGroup}>
          <label htmlFor="signup-confirm-password" className={authStyles.inputLabel}>
            Confirm Password
          </label>
          <div style={{ position: 'relative' }}>
            <i className={`fa fa-lock ${authStyles.inputIcon}`}></i>
            <input
              type="password"
              id="signup-confirm-password"
              className={authStyles.inputField}
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              required
              placeholder="Confirm your password"
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
              Creating Account...
            </>
          ) : (
            'Sign Up'
          )}
        </button>
      </form>
      {onSwitchToSignIn && (
        <div className={authStyles.switchForm}>
          <p className={authStyles.switchFormText}>
            Already have an account?
          </p>
          <button
            type="button"
            className={authStyles.switchFormButton}
            onClick={onSwitchToSignIn}
          >
            Sign in here
          </button>
        </div>
      )}
    </div>
  );
};

const SignUp: React.FC<SignUpProps> = (props) => {
  return (
    <SignUpContent {...props} />
  );
};

export default SignUp;