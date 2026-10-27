package com.example;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class HelloWorldTest {

    @Test
    @DisplayName("Should return the standard Hello, World! greeting")
    void shouldReturnDefaultGreeting() {
        assertEquals("Hello, World!", HelloWorld.getGreeting());
    }
}
