import { Pool } from 'pg';
declare const pool: Pool;
declare function createTables(): Promise<void>;
export { pool, createTables };
//# sourceMappingURL=migrate.d.ts.map