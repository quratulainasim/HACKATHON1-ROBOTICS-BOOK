import React from 'react';
import NavbarLayout from '@theme/Navbar/Layout';
import NavbarContent from '@theme/Navbar/Content';
import { useAuth } from '../components/AuthContext';
import styles from '../components/auth-navbar.module.css';

const CustomNavbarContent = () => {
  const { user, loading, signOut } = useAuth();

  const handleSignOut = async () => {
    await signOut();
  };

  return (
    <NavbarContent>
      {/* Add the auth component to the right side */}
      <div className="navbar__item navbar__items navbar__items--right">
        {loading ? (
          <span>Loading...</span>
        ) : user ? (
          <div className={styles.userDropdown}>
            <div className={styles.userAvatar}>
              {user.name ? user.name.charAt(0).toUpperCase() : user.email.charAt(0).toUpperCase()}
            </div>
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
    </NavbarContent>
  );
};

const CustomNavbar = () => {
  return (
    <NavbarLayout>
      <CustomNavbarContent />
    </NavbarLayout>
  );
};

export default CustomNavbar;