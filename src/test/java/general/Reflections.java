package general;

import java.lang.reflect.Field;

public class Reflections {
    public static <T> T getPrivateField(Object obj, String fieldName, Class<T> fieldType) throws NoSuchFieldException {
        try {
            final Field field = obj.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            return fieldType.cast(field.get(obj));
        } catch (IllegalAccessException e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to access private field: " + fieldName);
        }
    }
}
