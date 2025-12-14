// Type definitions for better-auth
declare module 'better-auth' {
  interface BetterAuthOptions {
    app?: {
      name?: string;
      baseURL?: string;
      siteURL?: string;
    };
    database: {
      url: string;
      type: string;
    };
    socialProviders?: object;
    emailAndPassword: {
      enabled: boolean;
      requireEmailVerification: boolean;
    };
    secret: string;
  }

  interface BetterAuthInstance {
    (req: any, res: any, next: any): void;
  }

  export function betterAuth(options: BetterAuthOptions): BetterAuthInstance;
}