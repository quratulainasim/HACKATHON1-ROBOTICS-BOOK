import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={styles.container}>
        <div className={styles.header}>
          <h1 className={clsx('hero__title', styles.title)}>
            {siteConfig.title}
          </h1>
          <p className={clsx('hero__subtitle', styles.subtitle)}>
            {siteConfig.tagline}
          </p>
        </div>

      

        {/* Author Section */}
        <section className={styles.authorSection}>
          <h2 className={styles.authorName}>Quratulain</h2>
          <p className={styles.authorCredentials}>
            MBA & MSc | Full Stack Developer | AI Engineer
          </p>

          
        </section>

        {/* Technologies Section */}
        <section className={styles.technologiesSection}>
          <h2 className={styles.technologiesTitle}>Technologies Covered</h2>
          <div className={styles.techGrid}>
            <div className={styles.techItem}>
              <div className={styles.techIcon}>🤖</div>
              <h3 className={styles.techName}>ROS 2</h3>
              <p className={styles.techDescription}>Robot Operating System for communication and control</p>
            </div>
            <div className={styles.techItem}>
              <div className={styles.techIcon}>🎮</div>
              <h3 className={styles.techName}>Gazebo</h3>
              <p className={styles.techDescription}>Physics simulation for robot testing and development</p>
            </div>
            <div className={styles.techItem}>
              <div className={styles.techIcon}>UNITY</div>
              <h3 className={styles.techName}>Unity</h3>
              <p className={styles.techDescription}>Visualization and Human-Robot Interaction</p>
            </div>
            <div className={styles.techItem}>
              <div className={styles.techIcon}>NVIDIA</div>
              <h3 className={styles.techName}>Isaac Sim</h3>
              <p className={styles.techDescription}>Photorealistic simulation and synthetic data generation</p>
            </div>
            <div className={styles.techItem}>
              <div className={styles.techIcon}>🧠</div>
              <h3 className={styles.techName}>LLM Integration</h3>
              <p className={styles.techDescription}>Large Language Models for task planning</p>
            </div>
            <div className={styles.techItem}>
              <div className={styles.techIcon}>🗣️</div>
              <h3 className={styles.techName}>Whisper</h3>
              <p className={styles.techDescription}>Speech recognition for voice commands</p>
            </div>
          </div>
        </section>

        {/* Buttons */}
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--secondary button--lg', styles.button)}
            to="/docs/intro/physical-ai-overview">
            Start Reading
          </Link>
          <Link
            className={clsx('button button--secondary button--lg', styles.button)}
            to="/docs/module-1-ros2/intro">
            Begin Module 1
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="A Comprehensive Guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
      </main>
    </Layout>
  );
}
