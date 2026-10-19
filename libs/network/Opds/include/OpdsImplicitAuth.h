#pragma once
#include <functional>
#include <string>

/**
 * Drive an OPDS OAuth implicit-grant login without a browser. Follows the
 * authentication document's "authenticate" URL, fills the first HTML login
 * form encountered with the stored credentials (carrying session cookies
 * across redirect hops, including federated SSO on another host), and
 * captures the access token from the opds://authorize callback redirect.
 *
 * Verified against Cantook/De Marque catalogs (Lirtuel + Samarcande SSO).
 * Only available on wolfSSL builds; returns false elsewhere.
 *
 * The chain is up to ten requests of 30 s each. `shouldAbort`, when set, is polled before each request and
 * inside it (the HTTP client's abort callback); a stop returns false.
 */
bool opdsImplicitAuthenticate(const std::string& authenticateUrl, const std::string& username,
                              const std::string& password, std::string& outToken,
                              const std::function<bool()>& shouldAbort = nullptr);
