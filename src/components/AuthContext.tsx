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

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);

  // FINAL WORKING CONFIG: Points to your live backend
  // Local dev → localhost (change port if yours is different)
  // Production → your deployed Railway server
  const authServerUrl =
    typeof window !== 'undefined' &&
    (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1')
      ? 'http://localhost:8001'  // ← Change this if your local auth runs on a different port (e.g., 5000, 3000)
      : 'https://hackathon1-robotics-book-production.up.railway.app';

  console.log('AuthProvider using authServerUrl:', authServerUrl);

  const fetchUser = async (authToken: string) => {
    try {
      const response = await fetch(`${authServerUrl}/api/auth/user`, {
        method: 'GET',
        headers: {
          Authorization: `Bearer ${authToken}`,
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();
        setUser(data.user);
      } else {
        localStorage.removeItem('auth-token');
        setToken(null);
        setUser(null);
      }
    } catch (error) {
      console.error('Error fetching user:', error);
      localStorage.removeItem('auth-token');
      setToken(null);
      setUser(null);
    }
  };

  const signIn = async (email: string, password: string) => {
    try {
      const response = await fetch(`${authServerUrl}/api/auth/sign-in/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Sign in failed');
      }

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
      const response = await fetch(`${authServerUrl}/api/auth/sign-up/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password, name }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Sign up failed');
      }

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
        await fetch(`${authServerUrl}/api/auth/sign-out`, {
          method: 'POST',
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        });
      } catch (error) {
        console.error('Sign out request failed:', error);
      }
    }

    localStorage.removeItem('auth-token');
    setToken(null);
    setUser(null);
  };

  const getUser = async () => {
    if (token) {
      try {
        const response = await fetch(`${authServerUrl}/api/auth/user`, {
          method: 'GET',
          headers: {
            Authorization: `Bearer ${token}`,
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

  // Initialize auth on mount
  useEffect(() => {
    const initAuth = async () => {
      const storedToken = localStorage.getItem('auth-token');
      if (storedToken) {
        setToken(storedToken);
        await fetchUser(storedToken);
      }
      setLoading(false);
    };

    initAuth();
  }, []);

  const value = {
    user,
    token,
    loading,
    signIn,
    signUp,
    signOut,
    getUser,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
