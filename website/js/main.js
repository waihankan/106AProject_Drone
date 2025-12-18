/**
 * Modular Drone Control System - Main JavaScript
 * Handles interactive features and smooth scrolling
 */

// Wait for DOM to be fully loaded
document.addEventListener('DOMContentLoaded', function() {

    // Smooth scrolling for anchor links
    const anchorLinks = document.querySelectorAll('a[href^="#"]');

    anchorLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            const href = this.getAttribute('href');

            // Skip empty or just # links
            if (href === '#' || href === '') {
                e.preventDefault();
                return;
            }

            const target = document.querySelector(href);

            if (target) {
                e.preventDefault();

                // Calculate offset for fixed header
                const headerOffset = 100;
                const elementPosition = target.getBoundingClientRect().top;
                const offsetPosition = elementPosition + window.pageYOffset - headerOffset;

                window.scrollTo({
                    top: offsetPosition,
                    behavior: 'smooth'
                });

                // Update active state in sidebar
                updateActiveSidebarLink(href);
            }
        });
    });

    // Highlight active section in sidebar on scroll
    const sections = document.querySelectorAll('section[id]');
    const sidebarLinks = document.querySelectorAll('aside a[href^="#"]');

    function updateActiveSidebarLink(activeHref = null) {
        sidebarLinks.forEach(link => {
            link.classList.remove('active', 'border-primary', 'text-primary');
            link.classList.add('border-transparent');
        });

        if (activeHref) {
            const activeLink = document.querySelector(`aside a[href="${activeHref}"]`);
            if (activeLink) {
                activeLink.classList.add('active', 'border-primary', 'text-primary');
                activeLink.classList.remove('border-transparent');
            }
        }
    }

    function highlightActiveSectionOnScroll() {
        let currentSection = '';
        const scrollPosition = window.pageYOffset + 150;

        sections.forEach(section => {
            const sectionTop = section.offsetTop;
            const sectionHeight = section.clientHeight;

            if (scrollPosition >= sectionTop && scrollPosition < sectionTop + sectionHeight) {
                currentSection = '#' + section.getAttribute('id');
            }
        });

        if (currentSection) {
            updateActiveSidebarLink(currentSection);
        }
    }

    // Throttle scroll event for performance
    let scrollTimeout;
    window.addEventListener('scroll', function() {
        if (scrollTimeout) {
            window.cancelAnimationFrame(scrollTimeout);
        }

        scrollTimeout = window.requestAnimationFrame(function() {
            highlightActiveSectionOnScroll();
        });
    });

    // Initial highlight
    highlightActiveSectionOnScroll();

    // Dark mode toggle (optional - can be added later)
    // This is a placeholder for future dark mode toggle functionality
    function toggleDarkMode() {
        document.documentElement.classList.toggle('dark');
        localStorage.setItem('darkMode', document.documentElement.classList.contains('dark'));
    }

    // Check for saved dark mode preference
    const savedDarkMode = localStorage.getItem('darkMode');
    if (savedDarkMode === 'true') {
        document.documentElement.classList.add('dark');
    }

    // Add fade-in animation to sections on scroll
    const observerOptions = {
        threshold: 0.1,
        rootMargin: '0px 0px -100px 0px'
    };

    const observer = new IntersectionObserver(function(entries) {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.style.opacity = '1';
                entry.target.style.transform = 'translateY(0)';
            }
        });
    }, observerOptions);

    // Observe all sections for fade-in effect
    sections.forEach(section => {
        section.style.opacity = '0';
        section.style.transform = 'translateY(20px)';
        section.style.transition = 'opacity 0.6s ease-out, transform 0.6s ease-out';
        observer.observe(section);
    });

    // Video lazy loading
    const videos = document.querySelectorAll('video');
    videos.forEach(video => {
        video.addEventListener('loadeddata', function() {
            video.style.opacity = '1';
        });
    });

    // External link handling
    const externalLinks = document.querySelectorAll('a[href^="http"]');
    externalLinks.forEach(link => {
        if (!link.hasAttribute('target')) {
            link.setAttribute('target', '_blank');
            link.setAttribute('rel', 'noopener noreferrer');
        }
    });

    // Console message
    console.log('%c🚁 Modular Drone Control System', 'color: #137fec; font-size: 20px; font-weight: bold;');
    console.log('%cDocumentation Website v1.0', 'color: #666; font-size: 12px;');
});

// Utility function to copy code blocks (optional enhancement)
function copyToClipboard(text) {
    if (navigator.clipboard) {
        navigator.clipboard.writeText(text).then(() => {
            console.log('Copied to clipboard');
        }).catch(err => {
            console.error('Failed to copy:', err);
        });
    }
}

// Export functions for potential external use
window.droneControlDocs = {
    copyToClipboard: copyToClipboard
};
