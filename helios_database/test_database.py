import os
import sys
import logging
from pathlib import Path

# Add the parent directory to the Python path
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir))

from config.database import engine, SessionLocal, Base
from models.robot import Robot

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def create_tables():
    """Create all database tables"""
    logger.info("Creating database tables...")
    Base.metadata.create_all(bind=engine)

def add_test_robots():
    """Add some test robots to the database"""
    db = SessionLocal()
    try:
        # Check if we already have robots
        existing_robots = db.query(Robot).count()
        if existing_robots > 0:
            logger.info(f"Database already contains {existing_robots} robots. Skipping test data creation.")
            return

        # Create test robots
        test_robots = [
            Robot(name="Helios-1", model="Mark I", battery_level=95.5),
            Robot(name="Helios-2", model="Mark I", battery_level=87.3),
            Robot(name="Helios-X", model="Prototype", battery_level=100.0),
        ]
        
        # Add robots to the session
        db.add_all(test_robots)
        db.commit()
        logger.info("Successfully added test robots to the database")
        
    except Exception as e:
        logger.error(f"Error adding test robots: {e}")
        db.rollback()
        raise
    finally:
        db.close()

def query_robots():
    """Query and display all robots"""
    db = SessionLocal()
    try:
        # Query all robots
        robots = db.query(Robot).all()
        logger.info("\nCurrent robots in database:")
        for robot in robots:
            logger.info(f"ID: {robot.id}, {robot.name} ({robot.model}) - Battery: {robot.battery_level}%")
            logger.info(f"Created at: {robot.created_at}, Last updated: {robot.updated_at}")
            logger.info("-" * 50)
            
    except Exception as e:
        logger.error(f"Error querying robots: {e}")
        raise
    finally:
        db.close()

def main():
    """Main function to run all tests"""
    try:
        # Create tables
        create_tables()
        
        # Add test data
        add_test_robots()
        
        # Query and display data
        query_robots()
        
        logger.info("Database test completed successfully!")
        
    except Exception as e:
        logger.error(f"Test failed: {e}")
        raise

if __name__ == "__main__":
    main() 