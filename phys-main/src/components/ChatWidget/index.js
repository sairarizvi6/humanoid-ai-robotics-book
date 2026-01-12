// // // src/components/ChatWidget/index.js
// import React, { useState, useRef, useEffect } from 'react';
// import styles from './styles.module.css';

// const ChatWidget = () => {
//   const [isOpen, setIsOpen] = useState(false);
//   const [messages, setMessages] = useState([
//     {
//       role: 'assistant',
//       content: '👋 Hi! I\'m your **Physical AI & Humanoid Robotics** assistant. I can help you with:\n\n• ROS 2 fundamentals\n• Gazebo simulation\n• NVIDIA Isaac Sim\n• Humanoid robot control\n• Vision-Language-Action models\n\nWhat would you like to learn about?'
//     }
//   ]);
//   const [input, setInput] = useState('');
//   const [isLoading, setIsLoading] = useState(false);
//   const messagesEndRef = useRef(null);
//   const inputRef = useRef(null);
//   const textareaRef = useRef(null);

//   const scrollToBottom = () => {
//     messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
//   };

//   useEffect(() => {
//     scrollToBottom();
//   }, [messages]);

//   useEffect(() => {
//     if (isOpen && inputRef.current) {
//       inputRef.current.focus();
//     }
//   }, [isOpen]);

//   // Auto-resize textarea
//   useEffect(() => {
//     if (textareaRef.current) {
//       textareaRef.current.style.height = 'auto';
//       textareaRef.current.style.height = textareaRef.current.scrollHeight + 'px';
//     }
//   }, [input]);

//   const formatMessage = (text) => {
//     // Simple markdown-like formatting
//     return text
//       .split('\n')
//       .map((line, i) => {
//         // Bold text
//         line = line.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
//         // Bullet points
//         if (line.trim().startsWith('•')) {
//           return `<div style="margin-left: 8px;">${line}</div>`;
//         }
//         return line;
//       })
//       .join('<br/>');
//   };

//   const sendMessage = async (e) => {
//     e.preventDefault();
    
//     if (!input.trim() || isLoading) return;

//     const userMessage = input.trim();
//     setInput('');
    
//     // Reset textarea height
//     if (textareaRef.current) {
//       textareaRef.current.style.height = 'auto';
//     }
    
//     // Add user message
//     setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
//     setIsLoading(true);

//     try {
//       const response = await fetch('https://phys-chatbot-api.vercel.app/chat', {
//         method: 'POST',
//         headers: {
//           'Content-Type': 'application/json',
//         },
//         body: JSON.stringify({ message: userMessage }),
//       });

//       if (!response.ok) {
//         throw new Error(`HTTP error! status: ${response.status}`);
//       }

//       const data = await response.json();
      
//       // Add assistant response
//       setMessages(prev => [...prev, { 
//         role: 'assistant', 
//         content: data.response 
//       }]);
//     } catch (error) {
//       console.error('Error:', error);
//       setMessages(prev => [...prev, { 
//         role: 'assistant', 
//         content: '❌ Sorry, I encountered an error connecting to the server. Please try again in a moment.' 
//       }]);
//     } finally {
//       setIsLoading(false);
//     }
//   };

//   const handleKeyDown = (e) => {
//     if (e.key === 'Enter' && !e.shiftKey) {
//       e.preventDefault();
//       sendMessage(e);
//     }
//   };

//   const quickQuestions = [
//     "What is ROS 2?",
//     "Explain Gazebo simulation",
//     "What are VLA models?",
//     "How do humanoid robots work?"
//   ];

//   const handleQuickQuestion = (question) => {
//     setInput(question);
//     if (inputRef.current) {
//       inputRef.current.focus();
//     }
//   };

//   return (
//     <>
//       {/* Chat Button */}
//       <button
//         className={styles.chatButton}
//         onClick={() => setIsOpen(!isOpen)}
//         aria-label="Toggle chat"
//       >
//         {isOpen ? (
//           <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
//             <line x1="18" y1="6" x2="6" y2="18" strokeLinecap="round"/>
//             <line x1="6" y1="6" x2="18" y2="18" strokeLinecap="round"/>
//           </svg>
//         ) : (
//           <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
//             <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" strokeLinecap="round" strokeLinejoin="round"/>
//             <circle cx="9" cy="10" r="0.5" fill="currentColor" strokeWidth="0"/>
//             <circle cx="12" cy="10" r="0.5" fill="currentColor" strokeWidth="0"/>
//             <circle cx="15" cy="10" r="0.5" fill="currentColor" strokeWidth="0"/>
//           </svg>
//         )}
//       </button>

//       {/* Chat Window */}
//       {isOpen && (
//         <div className={styles.chatWindow}>
//           <div className={styles.chatHeader}>
//             <div className={styles.headerContent}>
//               <div className={styles.botAvatar}>🤖</div>
//               <div>
//                 <h3 className={styles.headerTitle}>PhysAI Assistant</h3>
//                 <p className={styles.headerSubtitle}>Powered by RAG AI</p>
//               </div>
//             </div>
//             <button
//               className={styles.closeButton}
//               onClick={() => setIsOpen(false)}
//               aria-label="Close chat"
//             >
//               ×
//             </button>
//           </div>

//           <div className={styles.chatMessages}>
//             {messages.map((msg, idx) => (
//               <div
//                 key={idx}
//                 className={`${styles.message} ${styles[msg.role]}`}
//               >
//                 {msg.role === 'assistant' && (
//                   <div className={styles.messageAvatar}>🤖</div>
//                 )}
//                 <div 
//                   className={styles.messageContent}
//                   dangerouslySetInnerHTML={{ __html: formatMessage(msg.content) }}
//                 />
//                 {msg.role === 'user' && (
//                   <div className={styles.messageAvatar}>👤</div>
//                 )}
//               </div>
//             ))}
            
//             {isLoading && (
//               <div className={`${styles.message} ${styles.assistant}`}>
//                 <div className={styles.messageAvatar}>🤖</div>
//                 <div className={styles.messageContent}>
//                   <div className={styles.typingIndicator}>
//                     <span></span>
//                     <span></span>
//                     <span></span>
//                   </div>
//                 </div>
//               </div>
//             )}
            
//             {messages.length === 1 && !isLoading && (
//               <div style={{ 
//                 display: 'grid', 
//                 gridTemplateColumns: 'repeat(2, 1fr)', 
//                 gap: '8px',
//                 marginTop: '12px' 
//               }}>
//                 {quickQuestions.map((q, idx) => (
//                   <button
//                     key={idx}
//                     onClick={() => handleQuickQuestion(q)}
//                     style={{
//                       padding: '12px 14px',
//                       background: 'var(--ifm-color-emphasis-200)',
//                       border: '2px solid var(--ifm-color-emphasis-300)',
//                       borderRadius: '12px',
//                       fontSize: '13px',
//                       cursor: 'pointer',
//                       transition: 'all 0.2s ease',
//                       color: '#1e1b4b',
//                       textAlign: 'left',
//                       fontWeight: '600',
//                       lineHeight: '1.4'
//                     }}
//                     onMouseEnter={(e) => {
//                       e.target.style.transform = 'translateY(-2px)';
//                       e.target.style.boxShadow = '0 4px 12px rgba(99, 102, 241, 0.15)';
//                       e.target.style.borderColor = '#6366f1';
//                     }}
//                     onMouseLeave={(e) => {
//                       e.target.style.transform = 'translateY(0)';
//                       e.target.style.boxShadow = 'none';
//                       e.target.style.borderColor = 'var(--ifm-color-emphasis-200)';
//                     }}
//                   >
//                     {q}
//                   </button>
//                 ))}
//               </div>
//             )}
            
//             <div ref={messagesEndRef} />
//           </div>

//           <form className={styles.chatInput} onSubmit={sendMessage}>
//             <textarea
//               ref={(el) => {
//                 inputRef.current = el;
//                 textareaRef.current = el;
//               }}
//               value={input}
//               onChange={(e) => setInput(e.target.value)}
//               onKeyDown={handleKeyDown}
//               placeholder="Ask about robotics, ROS 2, simulation..."
//               disabled={isLoading}
//               className={styles.input}
//               rows={1}
//               style={{
//                 minHeight: '48px',
//                 maxHeight: '120px',
//                 overflow: 'auto',
//                 color: '#1e1b4b'
//               }}
//             />
//             <button
//               type="submit"
//               disabled={isLoading || !input.trim()}
//               className={styles.sendButton}
//               aria-label="Send message"
//             >
//               <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
//                 <line x1="22" y1="2" x2="11" y2="13" strokeLinecap="round" strokeLinejoin="round"/>
//                 <polygon points="22 2 15 22 11 13 2 9 22 2" strokeLinecap="round" strokeLinejoin="round"/>
//               </svg>
//             </button>
//           </form>
//         </div>
//       )}
//     </>
//   );
// };

// export default ChatWidget;

// src/components/ChatWidget/index.js
// src/components/ChatWidget/index.js
import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: '👋 Hi! I\'m your **Physical AI & Humanoid Robotics** assistant. I can help you with:\n\n• ROS 2 fundamentals\n• Gazebo simulation\n• NVIDIA Isaac Sim\n• Humanoid robot control\n• Vision-Language-Action models\n\nWhat would you like to learn about?'
    }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [user, setUser] = useState(null);
  const [showLogin, setShowLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const textareaRef = useRef(null);

  // Simple in-memory user storage
  const users = useRef({});

  // Check if user is logged in on mount
  useEffect(() => {
    const savedUser = sessionStorage.getItem('chatUser');
    if (savedUser) {
      setUser(JSON.parse(savedUser));
    }
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && inputRef.current && user) {
      inputRef.current.focus();
    }
  }, [isOpen, user]);

  // Auto-resize textarea
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = textareaRef.current.scrollHeight + 'px';
    }
  }, [input]);

  const formatMessage = (text) => {
    // Simple markdown-like formatting
    return text
      .split('\n')
      .map((line) => {
        // Bold text
        line = line.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
        // Bullet points
        if (line.trim().startsWith('•')) {
          return `<div style="margin-left: 8px;">${line}</div>`;
        }
        return line;
      })
      .join('<br/>');
  };

  const handleSignup = async (e) => {
    e.preventDefault();
    
    if (password.length < 6) {
      alert('Password must be at least 6 characters');
      return;
    }

    // Store user
    const userKey = email.toLowerCase();
    if (users.current[userKey]) {
      alert('User already exists! Please login.');
      setShowLogin(true);
      return;
    }

    const newUser = { email, name: name || email.split('@')[0], id: Date.now() };
    users.current[userKey] = { ...newUser, password };
    
    // Save to session
    sessionStorage.setItem('chatUser', JSON.stringify(newUser));
    sessionStorage.setItem('users', JSON.stringify(users.current));
    
    setUser(newUser);
    alert('Signup successful! Welcome, ' + newUser.name + '!');
  };

  const handleLogin = async (e) => {
    e.preventDefault();
    
    // Load users from session
    const savedUsers = sessionStorage.getItem('users');
    if (savedUsers) {
      users.current = JSON.parse(savedUsers);
    }

    const userKey = email.toLowerCase();
    const existingUser = users.current[userKey];

    if (!existingUser) {
      alert('User not found! Please sign up first.');
      setShowLogin(false);
      return;
    }

    if (existingUser.password !== password) {
      alert('Incorrect password!');
      return;
    }

    const userData = { email: existingUser.email, name: existingUser.name, id: existingUser.id };
    sessionStorage.setItem('chatUser', JSON.stringify(userData));
    
    setUser(userData);
    alert('Login successful! Welcome back, ' + userData.name + '!');
  };

  const handleLogout = () => {
    sessionStorage.removeItem('chatUser');
    setUser(null);
    setMessages([{
      role: 'assistant',
      content: '👋 Hi! I\'m your **Physical AI & Humanoid Robotics** assistant. I can help you with:\n\n• ROS 2 fundamentals\n• Gazebo simulation\n• NVIDIA Isaac Sim\n• Humanoid robot control\n• Vision-Language-Action models\n\nWhat would you like to learn about?'
    }]);
    setEmail('');
    setPassword('');
    setName('');
  };

  const sendMessage = async (e) => {
    e.preventDefault();
    
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput('');
    
    // Reset textarea height
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
    }
    
    // Add user message
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      const response = await fetch('https://phys-chatbot-api.vercel.app/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: userMessage }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      
      // Add assistant response
      setMessages(prev => [...prev, { 
        role: 'assistant', 
        content: data.response 
      }]);
    } catch (error) {
      console.error('Error:', error);
      setMessages(prev => [...prev, { 
        role: 'assistant', 
        content: '❌ Sorry, I encountered an error connecting to the server. Please try again in a moment.' 
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage(e);
    }
  };

  const quickQuestions = [
    "What is ROS 2?",
    "Explain Gazebo simulation",
    "What are VLA models?",
    "How do humanoid robots work?"
  ];

  const handleQuickQuestion = (question) => {
    setInput(question);
    if (inputRef.current) {
      inputRef.current.focus();
    }
  };

  return (
    <>
      {/* Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? (
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
            <line x1="18" y1="6" x2="6" y2="18" strokeLinecap="round"/>
            <line x1="6" y1="6" x2="18" y2="18" strokeLinecap="round"/>
          </svg>
        ) : (
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" strokeLinecap="round" strokeLinejoin="round"/>
            <circle cx="9" cy="10" r="0.5" fill="currentColor" strokeWidth="0"/>
            <circle cx="12" cy="10" r="0.5" fill="currentColor" strokeWidth="0"/>
            <circle cx="15" cy="10" r="0.5" fill="currentColor" strokeWidth="0"/>
          </svg>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <div className={styles.botAvatar}>🤖</div>
              <div>
                <h3 className={styles.headerTitle}>PhysAI Assistant</h3>
                <p className={styles.headerSubtitle}>
                  {user ? `👋 ${user.name}` : 'Please login to chat'}
                </p>
              </div>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          {/* Login/Signup Form */}
          {!user ? (
            <div style={{ padding: '24px' }}>
              <h3 style={{ marginBottom: '20px', fontSize: '18px', fontWeight: '600' }}>
                {showLogin ? '🔐 Login' : '✨ Create Account'}
              </h3>
              <form onSubmit={showLogin ? handleLogin : handleSignup}>
                {!showLogin && (
                  <input
                    type="text"
                    placeholder="Your Name"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                    style={{
                      width: '100%',
                      padding: '12px',
                      marginBottom: '10px',
                      border: '2px solid #e5e7eb',
                      borderRadius: '8px',
                      fontSize: '14px',
                      boxSizing: 'border-box'
                    }}
                  />
                )}
                <input
                  type="email"
                  placeholder="Email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                  style={{
                    width: '100%',
                    padding: '12px',
                    marginBottom: '10px',
                    border: '2px solid #e5e7eb',
                    borderRadius: '8px',
                    fontSize: '14px',
                    boxSizing: 'border-box'
                  }}
                />
                <input
                  type="password"
                  placeholder="Password (min 6 characters)"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  style={{
                    width: '100%',
                    padding: '12px',
                    marginBottom: '15px',
                    border: '2px solid #e5e7eb',
                    borderRadius: '8px',
                    fontSize: '14px',
                    boxSizing: 'border-box'
                  }}
                />
                <button
                  type="submit"
                  style={{
                    width: '100%',
                    padding: '12px',
                    background: '#4a90e2',
                    color: 'white',
                    border: 'none',
                    borderRadius: '8px',
                    fontSize: '14px',
                    fontWeight: '600',
                    cursor: 'pointer',
                    transition: 'background 0.2s'
                  }}
                  onMouseEnter={(e) => e.target.style.background = '#3a7bc8'}
                  onMouseLeave={(e) => e.target.style.background = '#4a90e2'}
                >
                  {showLogin ? 'Login' : 'Sign Up'}
                </button>
              </form>
              <p style={{ 
                marginTop: '15px', 
                textAlign: 'center', 
                fontSize: '13px',
                color: '#6b7280'
              }}>
                {showLogin ? "Don't have an account? " : "Already have an account? "}
                <button
                  onClick={() => {
                    setShowLogin(!showLogin);
                    setEmail('');
                    setPassword('');
                    setName('');
                  }}
                  style={{
                    background: 'none',
                    border: 'none',
                    color: '#4a90e2',
                    cursor: 'pointer',
                    fontWeight: '600',
                    fontSize: '13px'
                  }}
                >
                  {showLogin ? 'Sign Up' : 'Login'}
                </button>
              </p>
            </div>
          ) : (
            <>
              {/* Chat Messages */}
              <div className={styles.chatMessages}>
                {messages.map((msg, idx) => (
                  <div
                    key={idx}
                    className={`${styles.message} ${styles[msg.role]}`}
                  >
                    {msg.role === 'assistant' && (
                      <div className={styles.messageAvatar}>🤖</div>
                    )}
                    <div 
                      className={styles.messageContent}
                      dangerouslySetInnerHTML={{ __html: formatMessage(msg.content) }}
                    />
                    {msg.role === 'user' && (
                      <div className={styles.messageAvatar}>👤</div>
                    )}
                  </div>
                ))}
                
                {isLoading && (
                  <div className={`${styles.message} ${styles.assistant}`}>
                    <div className={styles.messageAvatar}>🤖</div>
                    <div className={styles.messageContent}>
                      <div className={styles.typingIndicator}>
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  </div>
                )}
                
                {/* Quick Questions - Only show on first message and when not loading */}
                {messages.length === 1 && !isLoading && (
                  <div style={{ 
                    display: 'grid', 
                    gridTemplateColumns: 'repeat(2, 1fr)', 
                    gap: '8px',
                    marginTop: '12px',
                    padding: '0 8px'
                  }}>
                    {quickQuestions.map((q, idx) => (
                      <button
                        key={idx}
                        onClick={() => handleQuickQuestion(q)}
                        style={{
                          padding: '12px 14px',
                          background: 'var(--ifm-color-emphasis-200)',
                          border: '2px solid var(--ifm-color-emphasis-300)',
                          borderRadius: '12px',
                          fontSize: '13px',
                          cursor: 'pointer',
                          transition: 'all 0.2s ease',
                          color: '#1e1b4b',
                          textAlign: 'left',
                          fontWeight: '600',
                          lineHeight: '1.4'
                        }}
                        onMouseEnter={(e) => {
                          e.target.style.transform = 'translateY(-2px)';
                          e.target.style.boxShadow = '0 4px 12px rgba(74, 144, 226, 0.15)';
                          e.target.style.borderColor = '#4a90e2';
                        }}
                        onMouseLeave={(e) => {
                          e.target.style.transform = 'translateY(0)';
                          e.target.style.boxShadow = 'none';
                          e.target.style.borderColor = 'var(--ifm-color-emphasis-300)';
                        }}
                      >
                        {q}
                      </button>
                    ))}
                  </div>
                )}
                
                <div ref={messagesEndRef} />
              </div>

              {/* Logout Button */}
              <div style={{
                padding: '12px 16px',
                borderTop: '1px solid #e5e7eb',
                display: 'flex',
                gap: '8px'
              }}>
                <button
                  onClick={handleLogout}
                  style={{
                    flex: 1,
                    padding: '8px',
                    background: '#4a90e2',
                    color: 'white',
                    border: 'none',
                    borderRadius: '6px',
                    fontSize: '13px',
                    fontWeight: '600',
                    cursor: 'pointer',
                    transition: 'background 0.2s'
                  }}
                  onMouseEnter={(e) => e.target.style.background = '#3a7bc8'}
                  onMouseLeave={(e) => e.target.style.background = '#4a90e2'}
                >
                  🚪 Logout
                </button>
              </div>

              {/* Chat Input */}
              <form className={styles.chatInput} onSubmit={sendMessage}>
                <textarea
                  ref={(el) => {
                    inputRef.current = el;
                    textareaRef.current = el;
                  }}
                  value={input}
                  onChange={(e) => setInput(e.target.value)}
                  onKeyDown={handleKeyDown}
                  placeholder="Ask about robotics, ROS 2, simulation..."
                  disabled={isLoading}
                  className={styles.input}
                  rows={1}
                  style={{
                    minHeight: '48px',
                    maxHeight: '120px',
                    overflow: 'auto',
                    color: '#1e1b4b'
                  }}
                />
                <button
                  type="submit"
                  disabled={isLoading || !input.trim()}
                  className={styles.sendButton}
                  aria-label="Send message"
                >
                  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <line x1="22" y1="2" x2="11" y2="13" strokeLinecap="round" strokeLinejoin="round"/>
                    <polygon points="22 2 15 22 11 13 2 9 22 2" strokeLinecap="round" strokeLinejoin="round"/>
                  </svg>
                </button>
              </form>
            </>
          )}
        </div>
      )}
    </>
  );
};

export default ChatWidget;