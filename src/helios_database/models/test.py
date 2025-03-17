from sqlalchemy import Column, Integer, String
from config.database import Base
from models.base import TimestampMixin

class YourModel(Base, TimestampMixin):
    __tablename__ = "your_table"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String)
    # Add more fields as needed
