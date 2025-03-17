from sqlalchemy import Column, Integer, String, Float, Boolean
from config.database import Base
from models.base import TimestampMixin

class Robot(Base, TimestampMixin):
    __tablename__ = "robots"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, nullable=False)
    model = Column(String)
    battery_level = Column(Float, default=100.0)
    is_active = Column(Boolean, default=True)
    
    def __repr__(self):
        return f"<Robot(name='{self.name}', model='{self.model}', battery_level={self.battery_level}%)>" 