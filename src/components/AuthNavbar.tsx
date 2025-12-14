import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from './AuthContext';
import styles from './auth-navbar.module.css';

const AuthNavbar: React.FC = () => {
  const { user, loading, signOut } = useAuth();
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  const handleSignOut = async () => {
    await signOut();
    setDropdownOpen(false);
  };

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setDropdownOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  if (loading) {
    return (
      <div className="navbar__item">
        <span>Loading...</span>
      </div>
    );
  }

  return (
    <div className="navbar__item">
      {user ? (
        <div className={`${styles.userDropdown} ${dropdownOpen ? styles.show : ''}`} ref={dropdownRef}>
          <button
            className={styles.userDropdownToggle}
            onClick={() => setDropdownOpen(!dropdownOpen)}
            aria-expanded={dropdownOpen}
            aria-haspopup="true"
          >
            <div className={styles.userAvatar}>
              {user.name ? user.name.charAt(0).toUpperCase() : user.email.charAt(0).toUpperCase()}
            </div>
          </button>
          <div className={styles.userDropdownContent}>
            <a href="/profile">Profile</a>
            <button onClick={handleSignOut}>Sign Out</button>
          </div>
        </div>
      ) : (
        <a href="/auth" className={styles.navbarSignInButton}>
          Sign In
        </a>
      )}
    </div>
  );
};

export default AuthNavbar;