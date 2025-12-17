declare module "better-auth" {
  export interface BetterAuthOptions {
    app?: {
      name?: string;
      baseURL?: string;
      siteURL?: string;
    };
    database: {
      url: string;
      type: "postgres";
    };
    emailAndPassword: {
      enabled: boolean;
      requireEmailVerification: boolean;
    };
    secret: string;
  }

  export type BetterAuthHandler = (
    req: any,
    res: any,
    next: any
  ) => void;

  export function betterAuth(
    options: BetterAuthOptions
  ): BetterAuthHandler;
}
