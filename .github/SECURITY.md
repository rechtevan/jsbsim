# Security Policy

## Supported Versions

Currently supported versions with security updates:

| Version | Supported          |
| ------- | ------------------ |
| Latest master | ✓ |
| JSBSim 1.2.x | ✓ |
| < 1.2.0 | ✗ |

## Reporting a Vulnerability

**Please do NOT report security vulnerabilities through public GitHub issues.**

Instead, please report security vulnerabilities via one of the following methods:

### Primary Method: GitHub Security Advisory

Use GitHub's [private vulnerability reporting feature](https://github.com/rechtevan/jsbsim/security/advisories):
1. Navigate to the Security tab in the repository
2. Click "Report a vulnerability"
3. Fill out the vulnerability report form with details

### Alternative Method: Email

If you prefer email communication:
- **Email**: [CONFIGURE EMAIL - Set fork owner's email or security@domain.com]
- **Subject**: [SECURITY] JSBSim Vulnerability Report

Please include in your report:
- **Description**: Detailed description of the vulnerability
- **Steps to Reproduce**: Clear instructions to reproduce the issue
- **Potential Impact**: Assessment of potential harm and affected versions
- **Suggested Fix**: Any proposed solution (if available)
- **Your Contact Info**: How to reach you for follow-up questions

### What to Expect

- **Initial Response**: Within 48 hours
- **Status Update**: Within 7 days with severity assessment and remediation plan
- **Resolution Timeline**: Varies by severity
  - Critical (CVSS 9.0-10.0): 30 days
  - High (CVSS 7.0-8.9): 60 days
  - Medium (CVSS 4.0-6.9): 90 days
  - Low (CVSS 0.1-3.9): Best effort

### Disclosure Policy

We follow responsible coordinated vulnerability disclosure practices:

1. **Private Development**: Security fixes are developed in private branches, not in public repositories
2. **Testing**: Fixes are thoroughly tested before release
3. **Public Advisory**: A security advisory is published on GitHub
4. **Version Release**: The fix is included in the next version release
5. **Credit**: The researcher is credited in release notes (unless they prefer anonymity)
6. **Timeline**: We request a reasonable disclosure period before public discussion

Please avoid public disclosure until we have released a fix or determined a timeline is not needed.

## Security Update Process

1. **Report Received**: Vulnerability is logged and assessed
2. **Triage**: Severity and impact are determined
3. **Fix Development**: A patch is developed in a private branch
4. **Testing**: Comprehensive testing ensures the fix works and doesn't introduce regressions
5. **Security Advisory**: A GitHub security advisory is created with CVE information (if applicable)
6. **Release**: The fix is included in a new version or patch release
7. **Announcement**: Release notes and security advisory are published

## Vulnerability Scope

### In Scope

Security issues we actively address:

- **Remote Code Execution (RCE)**: Arbitrary code execution via malicious input or scripts
- **Injection Attacks**: XML injection, path injection, command injection vulnerabilities
- **Memory Safety**: Buffer overflows, use-after-free, memory corruption
- **Path Traversal**: Unauthorized file system access via path manipulation
- **Denial of Service (DoS)**: Crashes or resource exhaustion from malicious input
- **Information Disclosure**: Unintended exposure of sensitive data or internal state
- **Authentication/Authorization**: Flaws in access control or permission validation
- **Cryptographic Issues**: Weak or incorrectly implemented cryptographic functions

### Out of Scope

These are outside the scope of this security policy:

- **Third-Party Dependencies**: Report issues in Expat, GeographicLib, SimGear, etc. directly to those projects
- **Physical Access**: Attacks requiring physical device access
- **Social Engineering**: Phishing, pretexting, or social attacks
- **Theoretical Vulnerabilities**: Without proof of concept or demonstrable impact
- **Build/Deployment Issues**: Issues in CI/CD pipelines, deployment scripts, or infrastructure
- **Performance Issues**: Unless they constitute denial of service

## Security Features and Practices

JSBSim implements several security mechanisms:

### Code Analysis
- **CodeQL**: Automated security scanning on every push and pull request
- **Static Analysis**: Regular code review for security issues
- **Type Safety**: C++17 with strict compilation flags

### Data Validation
- **XML Schema Validation**: Aircraft configuration files are validated against XML schemas
- **Path Validation**: File paths are sanitized to prevent traversal attacks
- **Input Bounds Checking**: Numeric inputs are validated for reasonable ranges

### Memory Safety
- **Valgrind Testing**: Regular memory testing to detect leaks and corruption
- **Buffer Protections**: Careful bounds checking in critical code paths
- **Standard Containers**: Use of STL containers to avoid manual memory management

### Configuration Security
- **Safe Parsing**: Expat-based XML parsing with security configurations
- **Sandboxing**: Aircraft configurations are isolated from each other
- **Versioning**: Configuration format versioning prevents compatibility attacks

## Known Vulnerabilities

See [GitHub Security Advisories](https://github.com/rechtevan/jsbsim/security/advisories) for a complete list of known vulnerabilities and their status.

## Security Best Practices for Users

When using JSBSim, follow these practices:

1. **Keep Updated**: Always use the latest version to receive security patches
2. **Validate Input**: Validate any external data before passing to JSBSim
3. **Review Scripts**: Carefully review simulation scripts before execution
4. **Sandbox Execution**: Run untrusted simulations in isolated environments if possible
5. **Report Issues**: Report any suspicious behavior or potential vulnerabilities
6. **Monitor Advisories**: Subscribe to GitHub security advisories for the repository

## Security Reporting Resources

- [GitHub Security Advisory](https://github.com/rechtevan/jsbsim/security/advisories)
- [GitHub Security Policy](https://docs.github.com/en/code-security/getting-started/adding-a-security-policy-to-your-repository)
- [OWASP Top 10](https://owasp.org/Top10/)
- [CWE/SANS Top 25](https://cwe.mitre.org/top25/)

## Credits and Acknowledgments

We deeply appreciate the security research community and individuals who help keep JSBSim secure through responsible disclosure. Security researchers who report valid vulnerabilities will be credited in release notes (unless they prefer to remain anonymous).

## Contact

For non-security questions or general inquiries, please use:
- **GitHub Issues**: For bug reports and feature requests
- **Discussions**: For general questions and collaboration

For security-specific concerns, please use the methods described in the "Reporting a Vulnerability" section.

---

**Last Updated**: 2025-11-15

This security policy applies to the rechtevan/jsbsim fork. For upstream JSBSim-Team security information, refer to the [JSBSim-Team/jsbsim repository](https://github.com/JSBSim-Team/jsbsim).
