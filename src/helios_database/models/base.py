from sqlalchemy import Column, DateTime
from sqlalchemy.sql import func

class TimestampMixin:
    """Timestamp mixin for models"""
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now()) 