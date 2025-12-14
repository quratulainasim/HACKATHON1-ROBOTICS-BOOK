import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  createdAt?: string;
  emailVerified?: boolean;
}

interface AuthContextType {
  user: User | null;
  token: string | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string, name: string) => Promise<void>;
  signOut: () => Promise<void>;
  getUser: () => Promise<User | null>;
}

// Create context with a default value that matches the interface
const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);

  console.log('AuthProvider mounted - initializing auth context');

  // Define functions first so they're available to useEffect
  const fetchUser = async (authToken: string) => {
    try {
      const response = await fetch('http://localhost:3001/api/auth/user', {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${authToken}`,
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();
        setUser(data.user);
      } else {
        // Token might be invalid, clear it
        localStorage.removeItem('auth-token');
        setToken(null);
        setUser(null);
      }
    } catch (error) {
      console.error('Error fetching user:', error);
      localStorage.removeItem('auth-token');
      setToken(null);
      setUser(null);
    } finally {
      // Only set loading to false after initial load, not during user fetches
      // setLoading(false); // Don't do this here as it's called during token refresh
    }
  };

  const signIn = async (email: string, password: string) => {
    try {
      const response = await fetch('http://localhost:3001/api/auth/sign-in/email', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Sign in failed');
      }

      // Store token and user
      const { user: userData, token: authToken } = data;
      localStorage.setItem('auth-token', authToken);
      setToken(authToken);
      setUser(userData);
    } catch (error) {
      console.error('Sign in error:', error);
      throw error;
    }
  };

  const signUp = async (email: string, password: string, name: string) => {
    try {
      const response = await fetch('http://localhost:3001/api/auth/sign-up/email', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password, name }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Sign up failed');
      }

      // Store token and user
      const { user: userData, token: authToken } = data;
      localStorage.setItem('auth-token', authToken);
      setToken(authToken);
      setUser(userData);
    } catch (error) {
      console.error('Sign up error:', error);
      throw error;
    }
  };

  const signOut = async () => {
    if (token) {
      try {
        await fetch('http://localhost:3001/api/auth/sign-out', {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        });
      } catch (error) {
        console.error('Sign out request failed:', error);
        // Continue with local sign out even if server request fails
      }
    }

    // Clear local storage and state
    localStorage.removeItem('auth-token');
    setToken(null);
    setUser(null);
  };

  const getUser = async () => {
    if (token) {
      try {
        const response = await fetch('http://localhost:3001/api/auth/user', {
          method: 'GET',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        });

        if (response.ok) {
          const data = await response.json();
          setUser(data.user);
          return data.user;
        }
      } catch (error) {
        console.error('Error getting user:', error);
      }
    }
    return null;
  };

  // Check for existing token on component mount
  useEffect(() => {
    const initAuth = async () => {
      try {
        console.log('Initializing auth, checking for stored token');
        const storedToken = localStorage.getItem('auth-token');
        if (storedToken) {
          console.log('Found stored token, fetching user');
          setToken(storedToken);
          await fetchUser(storedToken);
        } else {
          console.log('No stored token found');
        }
      } catch (error) {
        console.error('Error initializing auth:', error);
        // Clear invalid token
        localStorage.removeItem('auth-token');
        setToken(null);
        setUser(null);
      } finally {
        console.log('Auth initialization complete, setting loading to false');
        setLoading(false);
      }
    };

    initAuth();
  }, []);

  // Debug the values being provided
  const value = {
    user,
    token,
    loading,
    signIn,
    signUp,
    signOut,
    getUser,
  };

  console.log('AuthProvider providing value:', { user: user?.email, token: !!token, loading });

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    // This means the component is not wrapped with AuthProvider
    // We'll return a default object with disabled functions
    console.warn('Auth context not wrapped with AuthProvider - using default state');
    return {
      user: null,
      token: null,
      loading: true,
      signIn: async () => {
        console.error('Auth context not initialized - signIn unavailable');
        throw new Error('Authentication not initialized');
      },
      signUp: async () => {
        console.error('Auth context not initialized - signUp unavailable');
        throw new Error('Authentication not initialized');
      },
      signOut: async () => {
        console.error('Auth context not initialized - signOut unavailable');
        throw new Error('Authentication not initialized');
      },
      getUser: async () => {
        console.error('Auth context not initialized - getUser unavailable');
        return null;
      }
    };
  }
  return context;
};